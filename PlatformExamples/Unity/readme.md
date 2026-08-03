# Introduction

`RulrImport.cs` is a Unity Editor script that lets you load a calibrated camera from ofxRulr straight into a Unity `Camera` component (position, rotation and projection matrix all included). It's contributed by Nick Fox-Gieg.

Originally contributed by Nick Fox-Gieg ([@n1ckfg](https://github.com/n1ckfg), https://github.com/n1ckfg/ofxRulr).

# Exporting a camera from ofxRulr

* Select a `Item::View` (or `Item::Camera`) node in the inspector.
* Click the `Export for Unity...` button and save the file, e.g. `MyCamera-Unity.json`.
* This writes a small standalone JSON file shaped like:

```json
{ "OpenGL": { "projectionMatrix": [16 floats], "viewMatrix": [16 floats] } }
```

  Note this is a dedicated flat-array export, separate from the nested-array `"OpenGL"` block that also appears inside the regular node save files (`View_0.json` etc.) — `RulrImport.cs` expects the flat form, so use this button rather than a saved node file.

# Installing/using in Unity

* Add [Newtonsoft.Json for Unity](https://docs.unity3d.com/Packages/com.unity.nuget.newtonsoft-json@latest) to your project (`com.unity.nuget.newtonsoft-json`) — `RulrImport.cs` depends on it for JSON deserialization.
* Copy `RulrImport.cs` into an `Editor` folder in your Unity project's `Assets`.
* Select a `Camera` in the scene, right-click its component header (or use the component's ⋮ menu) and choose `Load Rulr Camera...`.
* Pick the JSON file exported above. The camera's transform and projection matrix are updated to match ofxRulr's calibration.

## Notes

* The script also exposes generic `Matrix4x4` decomposition helpers (`ExtractTranslationFromMatrix`, `ExtractRotationFromMatrix`, `ExtractScaleFromMatrix`, `DecomposeMatrix`, `SetTransformFromMatrix`, `TranslationMatrix`) that are reusable outside the camera-loading context.
* Unity's `Matrix4x4` flat indexer (`matrix[0..15]`) is column-major, matching OpenGL/glm's native storage order — the ofxRulr-side export writes floats in that same order.
