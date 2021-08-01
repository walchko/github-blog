import Network

func sleeps(sec: Double){
    usleep(UInt32(sec * 1000000))
}

class Kevin {
    var connection: NWConnection?

    init(ip: NWEndpoint.Host, port: NWEndpoint.Port) {
        
        self.connection = NWConnection(
            host: ip,
            port: port,
            using: .udp)
        
        self.connection?.stateUpdateHandler = { (newState) in
            switch (newState) {
            case .ready:
                print("ready")
            case .setup:
                print("setup")
            case .cancelled:
                print("cancelled")
            case .preparing:
                print("Preparing")
            default:
                print("waiting or failed")
                
            }
        }
        self.connection?.start(queue: .global())
    }

    func send(_ content: String) {
        let contentToSendUDP = content.data(using: String.Encoding.utf8)
        self.connection?.send(
            content: contentToSendUDP,
            completion: NWConnection.SendCompletion.contentProcessed({ NWError in
                if (NWError == nil) {
                    print("Data was sent to UDP")
                } else {
                    print("ERROR! \n \(NWError!)")
                }
        }))
    }

    func receive() {
        self.connection?.receiveMessage { (data, context, isComplete, error) in
            if (isComplete) {
                print("Receive is complete")
                if (data != nil) {
                    let backToString = String(decoding: data!, as: UTF8.self)
                    print("Received message: \(backToString)")
                } else {
                    print("Data == nil")
                }
            }
        }
    }
}

func range(_ upto: Int) -> [Int] {
    return Array(0 ..< upto)
}

var udp = Kevin(ip: "10.0.1.154", port: 9001)

for i in range(5) {
    udp.send("hello")
//    usleep(100000) // 0.1 sec
    sleeps(sec: 0.001)
    udp.receive()
    udp.receive()
    udp.receive()
    udp.receive()
//    usleep(100000) // 0.1 sec
//    sleeps(sec: 0.1)
    print("Loop: \(i)")
}

