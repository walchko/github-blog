---
title: JSON
date: 2 July 2021
image: "banner.jpg"
---

It is more complicated than python, but works. Here is a super basic example:

```swift
import SwiftUI
import PlaygroundSupport

/// Codable is alias for Encodable and Decodable
struct TestData: Codable {
    var accel: [Double]
    var id: Int
}

let encoder = JSONEncoder()
let decoder = JSONDecoder()

let data = TestData(accel: [1.0,-2.0,3.0], id: 1)
let jdata = try encoder.encode(data)
let data2 = try decoder.decode(TestData.self, from: jdata)


print(">> Orig: \(data)")
print(">> Encoded: \(jdata)")
print(">> Decoded: \(data2)")
```

# References

- [Encode/Decode JSON](https://www.raywenderlich.com/3418439-encoding-and-decoding-in-swift)
- Apple Docs: [Archives and Serialization](https://developer.apple.com/documentation/foundation/archives_and_serialization)
