# Instructions

This is the source code of the paper "Terrain Point Cloud Inpainting via Signal Decomposition".

# Distribution

Windows 10 or newer

# Dependencies

3rd party list:

* boost
* opencv
* pcl

You can use C++ package management tools such as [vcpkg](https://vcpkg.io/en/) and [nuget](https://www.nuget.org/) to quickly install dependencies.

If you already have vcpkg installed and the path of your vcpkg.exe is ```C:\vcpkg\vcpkg.exe```, then you can run ```install-3rd-parties.bat``` directly, otherwise you can install the dependency manually by following the steps below:

1. Open powershell or cmd in the vcpkg folder and enter the following command to integrate vcpkg into all visual studio projects:

``` 
vcpkg integrate install
```

2. Install 3rd parties:

``` 
vcpkg install boost:x64-windows 
vcpkg install opencv:x64-windows 
vcpkg install pcl:x64-windows 
vcpkg install pcl[surface-on-nurbs]:x64-windows 
```

# Run

If your visual studio 2022 (Community) uses the default installation path, which is ```C:\Program Files\Microsoft Visual Studio\2022\Community\```, you can run ```clean&build&run-sample.bat``` directly, and the inpainted point cloud file will be generated in the folder ```Result``` after the program runs.

# Result description

The output of this code is a binary point cloud file in ```ply``` format, which can be visualized by third-party software such as cloudcompare according to your needs.

