---
title: How to Request Authorization to Media
date: 10 Jul 2021
---

> **Important:**
> If the appropriate key is not present in your app’s Info.plist file when your app 
> requests authorization or attempts to use a capture device, the system terminates 
> your app.

```swift
switch AVCaptureDevice.authorizationStatus(for: .video) {
    case .authorized: // The user has previously granted access to the camera.
        self.setupCaptureSession()
    
    case .notDetermined: // The user has not yet been asked for camera access.
        AVCaptureDevice.requestAccess(for: .video) { granted in
            if granted {
                self.setupCaptureSession()
            }
        }
    
    case .denied: // The user has previously denied access.
        return

    case .restricted: // The user can't grant access due to restrictions.
        return
}
```

# References

- [Requesting Authorization for Media Capture on iOS](https://developer.apple.com/documentation/avfoundation/cameras_and_media_capture/requesting_authorization_for_media_capture_on_ios)
