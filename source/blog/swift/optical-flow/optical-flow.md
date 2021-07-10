---
title: Optical Flow
date: 10 July 2021
---

![](https://miro.medium.com/max/1164/1*-rPeODurrn4HV1S1nJfXAg.png)

```swift
let visionRequest = VNGenerateOpticalFlowRequest(targetedCIImage: previousImage, options: [:])
```

```swift
func session(_ session: ARSession, didUpdate frame: ARFrame) {
    guard currentBuffer == nil, case .normal = frame.camera.trackingState else {
        return
    }
    self.currentBuffer = frame.capturedImage
    let handler = VNImageRequestHandler(cvPixelBuffer: self.currentBuffer!, options: [:])

    let orientation = CGImagePropertyOrientation(rawValue: UInt32(UIDevice.current.orientation.rawValue))!
    requestHandler = VNGenerateOpticalFlowRequest(
        targetedCVPixelBuffer: self.currentBuffer!, 
        orientation: orientation, 
        options: [:], 
        completionHandler: { [self] c,b in
            if let p = (c.results?.first as? VNPixelBufferObservation)?.pixelBuffer {
                self.opticalFlowTexture = CVMetalTextureGetTexture(createTexture(
                    fromPixelBuffer: p, 
                    pixelFormat: .rg32Float, 
                    planeIndex: 0)!)
            }
    })
    visionQueue.async {
        do {
            defer { self.currentBuffer = nil }
            try handler.perform([self.requestHandler!])
        } catch {
            print("Error: Vision request failed with error \"\(error)\"")
        }
    }
}
```

# References

- Apple docs: [VNGenerateOpticalFlowRequest](https://developer.apple.com/documentation/vision/vngenerateopticalflowrequest)
- [How to use VNGenerateOpticalFlowRequest?](https://developer.apple.com/forums/thread/654934)
- [What’s New in the Vision Framework in iOS 14](https://heartbeat.fritz.ai/whats-new-in-the-vision-framework-in-ios-14-73d22a942ba5)
