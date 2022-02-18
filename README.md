# DepthCameraPluginProject
---
 *Copyright (C) 2021 Listed authors: [Hisham Iqbal](mailto:hi213@ic.ac.uk)*
 
 **This repository can not be copied and/or distributed without the express permission of the listed authors**

---

Unity Plugin for using research mode functionality in HoloLens 2. Modified based on [HoloLens2ForCV](https://github.com/microsoft/HoloLens2ForCV) and [HoloLens2-ResearchMode-Unity](https://github.com/petergu684/HoloLens2-ResearchMode-Unity), integrating some basic OpenCV functionality.

Custom DLL which tries to detect the presence of tools equipped with IR-reflective markers.

**Research software which is not intended for public distribution/use. Distributed based on author's discretion.**

This repo is formed of two main projects: 
- `HL2UnityPlugin`: which constructs a DLL that allows access to the Research Mode API on the HoloLens 2.
- `Unity_HLPluginProject`: a sample Unity project demonstrating how to interface with the DLL, as well as custom types that allow you to control the Transforms of Unity objects based on info from the DLL.

## Instructions adapted from  [HoloLens2-ResearchMode-Unity](https://github.com/petergu684/HoloLens2-ResearchMode-Unity):

Skeleton to wrap HoloLens 2 research mode api into Windows Runtime extension. 

To use it in Unity,
- Ensure the custom Nuget packages are installed within the plugin project to add OpenCV functionality
- Build the plugin project (ARM64,Release) and copy **ALL** .dll and .winmd files in `HL2UnityPlugin\ARM64\Release\HL2UnityPlugin` into `Assets/Plugins/WSA/ARM64` folder of your Unity project.
- Change the architecture in your Unity build settings to be ARM64.
- After building the Visual Studio .sln from Unity, go to `App/[Project name]/Package.appxmanifest` and add the restricted capability to the manifest file. 
- Add `xmlns:rescap="http://schemas.microsoft.com/appx/manifest/foundation/windows10/restrictedcapabilities" ` and `IgnorableNamespaces="uap uap2 uap3 uap4 mp mobile iot rescap"` to the `Package.appxmanifest` as follows:

![Package.appxmanifest example](./appmanifest.PNG?raw=true)
- Save the changes and deploy the solution to your HoloLens 2.


### Note (as written by [Peter Gu](https://github.com/petergu684/)):
- The reconstructed point cloud still has the offset problem as is described [here](https://github.com/microsoft/HoloLens2ForCV/issues/12) for object beyond 1m.
- To visualize the depth image, you need a grayscale shader applied to your preview plane. Example: [grayscale shader](https://github.com/qian256/HoloLensARToolKit/blob/master/HoloLensARToolKit/Assets/Sample/Grayscale.shader).
- For point cloud, current implementation only returns the reconstructed point cloud as a float array (in the format of x,y,z,x,y,z,...). If you want to visualize it, I find [this project](https://github.com/MarekKowalski/LiveScan3D-Hololens) is a good example.
- This project is mainly to show how to use Reseach Mode in Unity. I only provided implementation on AHAT camera image visualization and point cloud reconstruction (based on depth map of AHAT camera). Feel free to modify the code according to your own need.
- If you need a sample project to get started, you can refer to the sample `Unity_HLPluginProject` folder.
