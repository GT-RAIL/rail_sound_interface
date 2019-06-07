# RAIL Sound Interface

This package generates sounds for the robot. The package contains:

1. A [`sound_server`](#server) to play sounds, specifically robot beeps and text-to-speech, sent to it as [`sound_play/SoundRequestAction`](http://docs.ros.org/api/sound_play/html/action/SoundRequest.html) messages.
1. A Python [`sound_client`](#client) designed to send the sound requests. Any node that wants to play sounds is best served by importing the client and sending its requests through it. Theoretically, the client can also connect and play sounds to `sound_play`---the messages are received and responses generated---but there are bugs, such as the fact that there is no actual sound, which have yet to be resolved.


## Contents

- [Quickstart](#quickstart)
- [Client](#client)
- [Server](#server)
- [TTS Setup](#tts-setup)


## Quickstart

To get up and running with this package, ensure you have the common dependencies; then run the launch file from the demo

### Dependencies

The code in this repository needs: `sox`, `requests`, `psutil`, and `pydub`.

```bash
sudo apt install sox
sudo -H pip install requests psutil pydub
```

For [text-to-speech](#tts-setup), the package needs [`docker`](https://docs.docker.com/install/linux/docker-ce/ubuntu/) (recommended), or a local build of [Mary TTS](http://mary.dfki.de/) (not recommended).

### Demo

This assumes that the docker image of MaryTTS is being used

1. Launch the sound server: `roslaunch rail_sound_interface sound_interface.launch`
1. Run the client script to play sounds. (You might have to wait about 30 sec):
```bash
# Any of the following should work
rosrun rail_sound_interface client_script speak "The answer to life, the universe, and everything is 42"
rosrun rail_sound_interface client_script beep CHEERFUL
```

**Note**: After stopping the launch file, don't forget to shut down the TTS container running in the background! `docker stop marytts`


## Client

Sound requests to the `sound_server` are designed to be sent through the sound client, which can perform text-to-speech to generate temporary `wav` files, which are ultimately sent to the server, this package's or `sound_play`.

#### Usage

```python
# Import the sound client
from rail_sound_interface import SoundClient

# Initialize the client
client = SoundClient(
    server_name=None,
    beeps=None,    # (dict) a dictionary of beeps. `None` to use the package's own
    affects=None,  # (dict) a dictionary of affect modifiers for TTS. `None` to use the package's own
)
client.connect()

# Then use the client object
client.say(text, *args, **kwargs)
client.beep(beep_key, *args, **kwargs)
```

The parameters used to initialize the client object are:

- `server_name`: The name(space) of the ROS action server for the sound action server.
- `beeps`: A dictionary of beep string keys to `wav` file names---the latter are played when a beep with that particular key is requested. The default avilable beep `wav` files are defined in [`sounds/`](sounds/), with keys corresponding to the filename following the prefix `R2D2_`.
- `affects`: A dictionary of affects to functions that generate [SSML](https://www.w3.org/TR/speech-synthesis/) markup corresponding to the affect when provided the text. By default, the `SoundClient` has its own static functions of the form `make_*` for such markup, but they are, as of now, no-ops.

The client also uses ROS Params:

- `<server_name>/voice` (string): The name of the Mary TTS voice
- `<server_name>/speech_gain` (int): The amount to raise or lower the volume of the TTS output by

#### TODO

Among the things left to complete in this client are:

1. Fill in the SSML markup modifiers for the different predefined affects. Also include a quick guide to SSML
1. Expose all the different Mary TTS parameters that are available for speech synthesis (such as the docker server URL). Some are baked into the code.


## Server

The `sound_server` is an alternative to `sound_play`. It was made because:

1. It uses `sox` (`play`) to play sounds and not what looks like an unstable Python sound API that sometimes causes `sound_play` to tie up the `alsa` sound pipeline and crash all linux sounds. Additional benefits include:
    1. We don't keep pointers to the `alsa` sources/sinks around, so we are actually able to clean up the temporary files after we play them.
    1. We can correctly check if a source or sink is in use, which again prevents `alsa` crashes
1. It only presents an action server (and not the topic too) for playing sounds, making for a much better API

However, the server was a quick hack to some much needed problems, and there are many things that still remain to be done:

1. We cannot play multiple sounds in parallel yet, though the API was designed to support that, and the reason we use an `ActionServer` instead of a `SimpleActionServer` is to solely facilitate the parallel play capabilities.
1. As part of the lack of parallelism, we cannot specify sound priorities or overrides at this time.
1. We do not output to the ROS diagnostics (unlike `sound_play`)
1. We do not honour `sound_play`'s requests to loop, or the built-in sounds that are part of that API
1. We assume that TTS is the domain of the client, and not of the server. Perhaps such an assumption is desirable, in which case the remaining action item for this bullet point is to ensure that the API to this node reflects that.


## TTS Setup

In order to perform TTS, the package uses [Mary TTS](http://mary.dfki.de/).


### MARY: Docker installation (Recommended)

If you have docker running, simply run:

```bash
docker run -d --rm --name marytts -p 59125:59125 gtrail/marytts:dfki-prudence-hsmm
```

This will start up MARY TTS with the `dfki-prudence-hsmm` voice installed. The image tag is meant to denote the different voices.

If you ever need to build the docker image with a new voice (`<voice>`), then run:

```bash
docker build -t gtrail/marytts:<voice> --build-arg MARYTTS_VOICE=<voice> docker/marytts/
```

Once the image is built, you can change the tag used in the previous `docker run` command.


### MARY: Local installation

**Prerequisites**:

Detailed instructions for JAVA at [link](https://www.atlantic.net/hipaa-compliant-cloud-hosting-services/how-to-install-java-ubuntu-14-04/).

```bash
# Need Java 7+ and Maven 3+
sudo apt-get install python-software-properties
sudo add-apt-repository ppa:webupd8team/java
sudo apt-get update
sudo apt-get install maven oracle-java8-installer
```

**MARY TTS**

Installing MaryTTS (in a folder of your choice. For the rest of these instructions, I'm going to assume this folder is `$MARYTTS_ROOT`):

```bash
cd $MARYTTS_ROOT
git clone git@github.com:marytts/marytts.git
cd marytts
git checkout v5.2
mvn install
```

By default, I prefer to use the `dfki-prudence-hsmm` voice. However, you should play with the other voices at [this link](http://mary.dfki.de:59125/). To download the voice of your choice:

```bash
cd $MARYTTS_ROOT/target/marytts-5.2/bin/
./marytts-component-installer
```

Select the voice(s) you want to install and hit install. Now you're ready for TTS!

```bash
# Run the Mary TTS server. It's a good idea to have an alias for this.
$MARYTTS/target/marytts-5.2/bin/marytts-server
```
