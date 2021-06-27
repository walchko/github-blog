---
title: Using Bonjour
---

1. IPv4 (and IPv6) link-local addressing
1. Multicast Name Resolution (mDNS)
1. DNS Service Discovery (DNS-SD)

Bonjour: 244.0.0.51

## Test Listener

```
echo -n "Hello UDP-World" | nc -4u -w1 -localhost 1024
```

`echo` and netcat (`nc`) params:

- `n`: do not print trailing return
- `4u`: IPv4 using UDP
- `w1`: timeout of 1 second

## `dns-sd`

### Browse

```
$ dns-sd -B _http._tcp local  
Browsing for _http._tcp.local  
Timestamp     A/R Flags if Domain                    Service Type              Instance Name  
16:30:59.870  Add     3  6 local.                    _http._tcp.               My Cool Web App  
16:30:59.871  Add     3  6 local.                    _http._tcp.               Someone Else's Web Service  
16:30:59.871  Add     3  6 local.                    _http._tcp.               
```

### Lookup

```
$ dns-sd -L "My Cool Web App" _http._tcp local  
Lookup My Cool Web App._http._tcp.local  
16:31:52.678  My\032Cool\032Web\032App._http._tcp.local. can be reached at MyWebServer.local.:80 (interface 6)  
^C 
```

### Query IP Address

```
$ dns-sd -Q MyWebServer.local  
Timestamp     A/R Flags if Name                             T   C Rdata  
16:32:40.786  Add     2  6 MyWebServer.local.               1   1 169.254.45.209  
^C  
```

## UDP POSIX Example

```swift
import Foundation

class Server {

  let servicePort = "1234"

  func start() {
    print("Server starting...")

    let socketFD = socket(AF_INET6, //Domain [AF_INET,AF_INET6, AF_UNIX]
                          SOCK_STREAM, //Type [SOCK_STREAM, SOCK_DGRAM, SOCK_SEQPACKET, SOCK_RAW]
                          IPPROTO_TCP  //Protocol [IPPROTO_TCP, IPPROTO_SCTP, IPPROTO_UDP, IPPROTO_DCCP]
                          )//Return a FileDescriptor -1 = error
    if socketFD == -1 {
      print("Error creating BSD Socket")
      return
    }

    var hints = addrinfo(
      ai_flags: AI_PASSIVE,       // Assign the address of the local host to the socket structures
      ai_family: AF_UNSPEC,       // Either IPv4 or IPv6
      ai_socktype: SOCK_STREAM,   // TCP
      ai_protocol: 0,
      ai_addrlen: 0,
      ai_canonname: nil,
      ai_addr: nil,
      ai_next: nil)

    var servinfo: UnsafeMutablePointer<addrinfo>? = nil
    let addrInfoResult = getaddrinfo(
      nil,                        // Any interface
      servicePort,                   // The port on which will be listenend
      &hints,                     // Protocol configuration as per above
      &servinfo)

    if addrInfoResult != 0 {
      print("Error getting address info: \(errno)")
      return
    }

    let bindResult = bind(socketFD, servinfo!.pointee.ai_addr, socklen_t(servinfo!.pointee.ai_addrlen))

    if bindResult == -1 {
      print("Error binding socket to Address: \(errno)")
      return
    }

    let listenResult = listen(socketFD, //Socket File descriptor
                              8         // The backlog argument defines the maximum length the queue of pending connections may grow to
    )

    if listenResult == -1 {
      print("Error setting our socket to listen")
      return
    }

    while (true) {
      let MTU = 65536
      var addr = sockaddr()
      var addr_len :socklen_t = 0

      print("About to accept")
      let clientFD = accept(socketFD, &addr, &addr_len)
      print("Accepted new client with file descriptor: \(clientFD)")

      if clientFD == -1 {
        print("Error accepting connection")
      }

      var buffer = UnsafeMutableRawPointer.allocate(byteCount: MTU,alignment: MemoryLayout<CChar>.size)

      while(true) {
        let readResult = read(clientFD, &buffer, MTU)

        if (readResult == 0) {
          break;  // end of file
        } else if (readResult == -1) {
          print("Error reading form client\(clientFD) - \(errno)")
          break;  // error
        } else {
          //This is an ugly way to add the null-terminator at the end of the buffer we just read
          withUnsafeMutablePointer(to: &buffer) {
                $0.withMemoryRebound(to: UInt8.self, capacity: readResult + 1) {
                    $0.advanced(by: readResult).assign(repeating: 0, count: 1)
                }
          }
          let strResult = withUnsafePointer(to: &buffer) {
            $0.withMemoryRebound(to: CChar.self, capacity: MemoryLayout.size(ofValue: readResult)) {
              String(cString: $0)
            }
          }
          print("Received form client(\(clientFD)): \(strResult)")
          write(clientFD, &buffer, readResult)
        }
      }
    }
  }

}
```

## TCP Socket Example

```swift
class TcpSocket {
    
    static let PORT: NWEndpoint.Port = 33120
    var connection: NWConnection!
    
    func connectToTcp(ipAddress: NWEndpoint.Host) {
        let queue = DispatchQueue(label: "TCP Client Queue")
        
        let tcp = NWProtocolTCP.Options.init()
        tcp.noDelay = true
        let params = NWParameters.init(tls: nil, tcp: tcp)
        connection = NWConnection(to: NWEndpoint.hostPort(host: ipAddress, port: TcpSocket.PORT), using: params)
        
        connection.stateUpdateHandler = { (newState) in

            switch (newState) {
            case .ready:
                print("Socket State: Ready")
                self.send()
                self.receive()
            default:
            }
        }
        connection.start(queue: queue)
    }
    
    func receive() {
        connection.receiveMessage { (data, context, isComplete, error) in
            if (isComplete) {
  print("Receive is complete, count bytes: \(data!.count)")
                if (data != nil) {
                    print(data!.byteToHex())
                } else {
                    print("Data == nil")
                }
            }
        }
    }
    
    func send() {
        let content: Data = „DummyData“.hexToData()
        connection.send(content: content, completion: NWConnection.SendCompletion.contentProcessed(({ (NWError) in
            if (NWError == nil) {
                print("Data was sent to TCP destination ")
            } else {
                print("ERROR! Error when data (Type: Data) sending. NWError: \n \(NWError!)")
            }
        })))
    }
}
```

# References

- stackexchange: [How to Use Bonjour](https://serverfault.com/questions/118652/how-to-use-bonjour)
- IANA: [Multicast addresses](https://www.iana.org/assignments/multicast-addresses/multicast-addresses.xhtml)
- [Swift UDP Listener](https://itnext.io/udp-listener-in-swift-1e4a0c0aa461)
- apple.developer: [How to Use Sockets](https://developer.apple.com/forums/thread/120486)
- github: [Echo Server](https://github.com/rderik/echoSocketServer)
