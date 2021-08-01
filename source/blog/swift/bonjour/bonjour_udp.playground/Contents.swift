import Foundation
import Network
import Combine

//let sendToScreen = PassthroughSubject <String,Never>()

/// Need to add to info.plist
/// - Privacy, local network
/// - Bonjour with a string of _whack._udp (which is the type below)
class Connect: NSObject {
    private var talking: NWConnection?
    private var listening: NWListener?
    
    func listenUDP(port: NWEndpoint.Port) {
        do {
            self.listening = try NWListener(using: .udp, on: port)
            self.listening?.stateUpdateHandler = {(newState) in
                switch newState {
                case .ready:
                    print("ready")
                default:
                    break
                }
            }
            self.listening?.newConnectionHandler = {(newConnection) in
                newConnection.stateUpdateHandler = {newState in
                    switch newState {
                    case .ready:
                        print("new connection")
                        self.receive(on: newConnection)
                    default:
                        break
                    }
                }
                newConnection.start(queue: DispatchQueue(label: "new client"))
            }
        } catch {
            print("unable to create listener")
        }
        self.listening?.start(queue: .main)
    }
    
    func receive(on connection: NWConnection) {
        connection.receiveMessage { (data, context, isComplete, error) in
            if let cont = context {
                print(cont)
            }
            if let error = error {
                print(error)
                return
            }
            if let data = data, !data.isEmpty {
                let backToString = String(decoding: data, as: UTF8.self)
                print("b2S",backToString)
//                DispatchQueue.main.async {
//                    sendToScreen.send(backToString)
//                }
            }
        }
    }
    
    func connectToUDP(hostUDP:NWEndpoint.Host,portUDP:NWEndpoint.Port) {
        self.talking = NWConnection(host: hostUDP, port: portUDP, using: .udp)
        self.talking?.stateUpdateHandler = { (newState) in
            switch (newState) {
            case .ready:
                break
            default:
                break
            }
        }
        self.talking?.start(queue: .main)
    }
    
    func sendUDP(_ content: String) {
        let contentToSendUDP = content.data(using: String.Encoding.utf8)
        self.talking?.send(content: contentToSendUDP, completion: NWConnection.SendCompletion.contentProcessed(({ (NWError) in
            if (NWError == nil) {
            // code
            } else {
                print("ERROR! Error when data (Type: String) sending. NWError: \n \(NWError!) ")
            }
        })))
    }
    
    func bonjourUDP(name: String) {
        do {
            self.listening = try NWListener(using: .udp)
            self.listening?.service = NWListener.Service(
                name:name,
                type: "_whack._udp",
                domain: nil,
                txtRecord: nil)
            self.listening?.stateUpdateHandler = {(newState) in
                switch newState {
                case .ready:
                    print("ready")
                default:
                    break
                }
            }
            self.listening?.serviceRegistrationUpdateHandler = { (serviceChange) in
                switch(serviceChange) {
                case .add(let endpoint):
                    switch endpoint {
                    case let .service(name, _, _, _):
                        print("Service ",name)
                    default:
                        break
                    }
                default:
                    break
                }
            }
            self.listening?.newConnectionHandler = {(newConnection) in
                newConnection.stateUpdateHandler = {newState in
                    switch newState {
                    case .ready:
                        print("new connection")
                        self.receive(on: newConnection)
                    default:
                        break
                    }
                }
                newConnection.start(queue: .main) // ????
            }
        } catch {
            print("unable to create listener")
        }
        self.listening?.start(queue: .main) // ???
    }
    
    func bonjourToUDP(name: String) {
        let svc: NWEndpoint = .service(
            name: name,
            type: "_whack._udp",
            domain: "local",
            interface: nil)
        self.talking = NWConnection(to: svc, using: .udp)
        self.talking?.stateUpdateHandler = { (newState) in
            switch (newState) {
            case .ready:
                print("ready to send \(self.talking!.endpoint)")
                print("\(self.talking!.endpoint)")
            default:
                break
            }
        }
        self.talking?.start(queue: .main)
    }
}

let communication = Connect()
communication.bonjourUDP(name: "whack")
communication.bonjourToUDP(name: "whack")
communication.sendUDP("pong\n")
