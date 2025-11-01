# ATAK Argonaut Plugin

Custom ATAK Plugin using NoviTAK. Check the wiki for docs.

## Installation

### Host Part (dev)

1. Install Android Studio
2. Login `tak.gov`
3. Go to `tak.gov/products/atak-civ`
4. Scroll down to `Downloadable Resources` and switch to `Developer Resources` tab.

![Screenshot from 2024-02-20 10-37-55](https://github.com/novitai/atak_custom_plugin/assets/138449715/68d7a19d-9e0a-4c08-b972-ae5800889e55)

5. Download and unzip `ATAK-CIV-5.0.0.18-SDK`
6. Go to `ATAK-CIV-5.0.0.18-SDK/samples` folder and clone this repo in there.
7. Open this repo with Android Studio and let it arrange the environment.
8. Install an older version of JDK that is compatible with the ATAK CI CD, such as JDK 11.
   - In Mac, this can be done using the Homebrew package manager. 
     - Install it by running `/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"` 
     - Use it to install JDK 11 by running `brew install --cask zulu@11`
10. Select the JDK version to use from `Android Studio -> Settings -> Build, Execution, Deployment -> Build Tools -> Gradle`
11. It should set the run config such that the `Run` option is enabled. If not, go to `app -> Edit Configurations... -> General -> Module` and use `atak_custom_plugin.app.main`.
12. Go to `Build -> Select Build Variant` and set `civDebug` for `Active Build Variant`.

### Mobile Part

1. Obtain an Android Device.
2. Download and install the ATAK Debug APK from [here](https://cloud.skybase.ai/index.php/s/4GZQjGyagMFJnio). Alternatively, it is available as "atak.apk" within the downloaded SDK folder.
3. Delete current ATAK app in your device if dependency conflict error occurs.
4. Open ATAK in your device and run the project from Android Studio.
