## Example folder contents

The project **Hydroponics** contains one source file in C language [main.c](/main/main.c). The file is located in folder [main](main).
And it's modules located in(/modules). All header files will be in /includes.

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  This is the file you are currently reading
```