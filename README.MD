# SFML examples

A series of small SFML example programs to help me learn my way around.

The individual targets mostly illustrate a single feature.

These are likely to change without warning and any or all of them may fail to build or run.

Tha top level CMakeLists.txt file should include any additions. Have a look for how this is done.

Take care that the working directory of the compiled executable can see the assets as a sub-directory.

No idea if this builds in Windows. Let me know. Make a pull request to offer changes for building in windows if needed.

---

## Prerequisites:

## Windows

Windows is a bit of a pain to get set up with a compiler but will already have the key libraries needed by SFML.

Get your basic tools:

https://www.youtube.com/watch?v=7sg88N_bDI0

- Install 7-zip (https://www.7-zip.org/download.html) or equivalent
- Install Git for Windows (https://git-scm.com/download/win) - accept defaults but check editor
- Install CMAKE (https://cmake.org/download/). Accept defaults, for all users, add to path
- Install VSCode (https://code.visualstudio.com). Accept defaults, add to path, add 'open with' options
- Add the C/C++ extensions for VSCode (https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools). Ignore
  any messages about setting up the default compiler

Add a compiler

- Install MinGW. You must get the correct version as shown on the SFML page
  at https://www.sfml-dev.org/download/sfml/2.6.1/
- Download the compiler zip file.
- Extract the contents
- Move the mingw64 directory to the root of drive C (It can go anywhere but this is most common)
- Add the path to your MinGW-w64 bin folder to the Windows PATH environment variable by using the following steps:
    - In the Windows search bar, type Settings to open your Windows Settings.
    - Search for Edit environment variables for your account.
    - In your User variables, select the Path variable and then select Edit.
    - Select New and add the MinGW-w64 destination folder you recorded during the installation process to the list.
      If you used the default settings above, then this will be the path: C:\msys64\bin.
    - Select OK to save the updated PATH. You will need to reopen any console windows for the new PATH location to
      be available
    - Open a new command prompt and check the setup is correct by entering the command `gcc --version`
    -
  Microsoft have a page about getting started with mingw in VSCode:
    - https://code.visualstudio.com/docs/cpp/config-mingw

#### Get sfml

_it must match the compiler_ that is the MinGW 64 bit version
It looks like Windows already has DLLs for things like openal, vorbis and flac.

## Linux

Basic tools

- Install git.
- Install CMake.
- For the compiler, you can just install a basic GCC.
    - For ubuntu based systems this is just `sudo apt install build-essential`
- Install VSCode in essentially the same way as you might for Windows - see above
- You will need some additional libraries and headers. Again, for a Ubuntu-based syste:
    - `sudo apt install libudev-dev`
    - `sudo apt install libopenal-dev`
    - `sudo apt install libvorbis-dev`
    - `sudo apt install libflac-dev`

## Mac

Help me out here. I do not have a Mac.

---

## Running the examples

This is not the place to go through the entire business of configuring and building the examples. I will try and find
some good tutorials on Youtube. Or you could find some and get the links included in this document.

