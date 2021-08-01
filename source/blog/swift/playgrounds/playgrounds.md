---
title: Swift Playgrounds
date: 9 July 2021
---

So playgrounds *kind of* sound like Jupyter notebooks, but they aren't as good.

| Feature      | Swift | Jupyter |
|--------------|-------|---------|
| Markdown     | ✔️     | ✔️       |
| LaTex        |       | ✔️       |
| `numpy`      |       | ✔️       |
| `matplotlib` |       | ✔️       |
| `opencv`     |       | ✔️       |
| `serial`     |       | ✔️       |
| `matplotlib` |       | ✔️       |
| `protobuf`,`msgpack`,`pickle` |       | ✔️       |

- **Note:** Example python libs are listed for *similar* capabilities like: `numpy` extensive scientific math routines.

## Super Simple Example

```swift
import Swift
import PlaygroundSupport

struct MyView: View {
    func body(): some View {
        // ...
    }
}

PlaygroundPage.current.liveView = myView()
```

## Markdown

You can use standard Markdown syntax inside `/*: .. */ and additionally:

- Xcode strips out excess whitespace, use HTML to force line breaks.
- Callouts provide special formatting:
    - `- important:` highlights text in red
    - `- note:`
    - `- experiment:` 
- Markdown files in your playground can be accessed by name: [Home](Intro)
- Moving between pages with: `[< Previous](@previous)` and `[Next >](@next)` 
    - these will navigate using the XML table of contents 

## XML Table of Contents

Create a directory `Pages` and put an XML file inside it with the names of
the pages:

```xml
<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<playground version='6.0' target-platform='ios' display-mode='rendered'>
    <pages>
        <page name='Introduction'/>
        <page name='Derived collections of enum cases'/>
        <page name='Warning and error diagnostic directives'/>
        <page name='Dynamic member look up'/>
        <page name='Enhanced conditional conformances'/>
        <page name='In-place collection element removal'/>
        <page name='Boolean toggling'/>
    </pages>
</playground>
```

## Cool Playgrounds

- SceneKit: [3D rotating Earth](https://github.com/cl7/Astronomy)

# References

- [Accessing Camera from Playground](https://medium.com/@moazzamtahir81/how-to-access-camera-on-swift-playground-with-custom-subview-f103c047847b)
- [WWDC21 Videos](https://developer.apple.com/videos/)
- [How to create interactive Swift playgrounds](https://www.hackingwithswift.com/articles/78/how-to-create-interactive-swift-playgrounds)
