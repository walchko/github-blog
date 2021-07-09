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

## Cool Playgrounds

- SceneKit: [3D rotating Earth](https://github.com/cl7/Astronomy)

# References

- [Accessing Camera from Playground](https://medium.com/@moazzamtahir81/how-to-access-camera-on-swift-playground-with-custom-subview-f103c047847b)
- [WWDC21 Videos](https://developer.apple.com/videos/)
