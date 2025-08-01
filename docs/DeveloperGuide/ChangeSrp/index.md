# Change SRP

## What's SRP?
SRP (Scriptable Render Pipeline) is a programmable rendering architecture provided by Unity that allows developers to customize rendering behavior based on project requirements. Unity officially offers two main SRP implementations:

- URP (Universal Render Pipeline): Optimized for cross-platform performance, suitable for most projects.
- HDRP (High Definition Render Pipeline): Prioritizes high visual fidelity, ideal for high-end platforms and photorealistic projects.

## How to Check the Current Render Pipeline
You can check which render pipeline your project is currently using by following these steps:

1. Open the Unity Editor and go to Edit > Project Settings > Graphics

![Graphics Settings](image_0.png)

1. Look at the Set Default Render Pipeline Asset field:

    - If it shows a HDRenderPipelineAsset, the project is using HDRP

    ![Graphics Settings](image_2.png)
    
    - If it shows a UniversalRenderPipelineAsset, the project is using URP
    ![Graphics Settings](image_1.png)

## Change rendering pipeline
In this project, both HDRP and URP scenes are preconfigured separately.

=== "URP to HDRP"
    1. Change the `Default Render Pipeline` in Edit -> ProjectSettings -> Graphics, and set it to `HDRenderPipelineAsset`.
    ![Graphics Settings](image_7.png)

    1. Open the AutowareSimulationDemo scene.
    ![Graphics Settings](image_6.png)

    1. Add Scripting Define Symbols  
    After switching the render pipeline, you need to change the scripting define symbol from URP to HDRP to enable HDRP-specific conditional compilation.  
        - In the Unity Editor, go to Edit -> Project Settings -> Player

        - In the right panel, expand Other Settings

        - Find the Scripting Define Symbols field under the Script Compilation section

        - Remove URP and add HDRP
        ![Graphics Settings](image_9.png)

    1. Restart the Unity Editor after changing the render pipeline.
        - Especially when switching from URP to HDRP, the following error may occur:
        ```
        Exception: Invalid import, you are importing a texture handle that wraps a RenderTargetIdentifier. The render graph can't know the properties of these textures so please use the ImportTexture overload that takes a RenderTargetInfo argument instead.
        ```
        - This is caused by leftover SRP cache and materials not being refreshed.

        - Scene materials may appear broken (e.g., white), even without error logs when switching from HDRP to URP.

        - Restarting Unity Editor helps avoid unknown issues and ensures proper scene rendering.

=== "HDRP to URP"
    1. Change the `Default Render Pipeline` in Edit -> ProjectSettings -> Graphics, and set it to `UniversalRenderPipelineAsset`.
    ![Graphics Settings](image_3.png)

    1. Open the AutowareSimulationURPDemo scene.
    ![Graphics Settings](image_4.png)
    
    1. Add Scripting Define Symbols  
    After switching the render pipeline, you need to change the scripting define symbol from HDRP to URP to enable URP-specific conditional compilation.  
        - In the Unity Editor, go to Edit -> Project Settings -> Player

        - In the right panel, expand Other Settings

        - Find the Scripting Define Symbols field under the Script Compilation section

        - Remove HDRP and add URP
        ![Graphics Settings](image_8.png)
    
    1. Restart the Unity Editor after changing the render pipeline.
        - Especially when switching from URP to HDRP, the following error may occur:
        ```
        Exception: Invalid import, you are importing a texture handle that wraps a RenderTargetIdentifier. The render graph can't know the properties of these textures so please use the ImportTexture overload that takes a RenderTargetInfo argument instead.
        ```
        - This is caused by leftover SRP cache and materials not being refreshed.

        - Scene materials may appear broken (e.g., white), even without error logs when switching from HDRP to URP.

        - Restarting Unity Editor helps avoid unknown issues and ensures proper scene rendering.

    1. Check the lighting configuration:
    ![Graphics Settings](image_5.png)