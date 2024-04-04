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

The following shows 3rd party library download steps if you have installed vcpkg:

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




