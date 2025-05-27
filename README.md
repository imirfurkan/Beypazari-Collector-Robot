## How to build

1. Install PlatformIO extension on VSCode
2. Clone this repository
3. Open with VSCode + PlatformIO extension
4. Click “Build”

## Styling

Go to VSCode settings, search for "format on save" and make sure that it is ticked.
This makes use the .clang-format upon saving a file.

## About the codes

The individual components of the system and their codes are handled in lib with their respective .cpp and .h files.
Over time, these are combined into main.cpp
Under test folder, there are test files for certain codes which may or may not be outdated. End of the void loop, there are test codes that can be directly pasted into main.cpp to run the test.