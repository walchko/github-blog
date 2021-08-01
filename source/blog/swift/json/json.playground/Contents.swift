import SwiftUI
import PlaygroundSupport

/// Codable is alias for Encodable and Decodable
struct TestData: Codable {
    var accel: [Double]
    var id: Int
}

let encoder = JSONEncoder()
let decoder = JSONDecoder()

let data = TestData(accel: [1.10,-2.20,3.30], id: 1)
let jdata = try encoder.encode(data)
let data2 = try decoder.decode(TestData.self, from: jdata)


print(">> Orig: \(data)")
print(">> Encoded[\(jdata)]: \(String(data: jdata, encoding: .utf8)!)")
print(">> Decoded: \(data2)")
