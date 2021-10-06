//
//  CircleInstanceController.swift
//  RealDeviceMap
//
//  Created by Florian Kostenzer on 30.09.18.
//

import Foundation
import PerfectLib
import PerfectThread
import PerfectMySQL

class CircleInstanceController: InstanceControllerProto {

    enum CircleType {
        case pokemon
        case smartPokemon
        case raid
        case smartStaticQuest
    }

    public private(set) var name: String
    public private(set) var minLevel: UInt8
    public private(set) var maxLevel: UInt8
    public private(set) var accountGroup: String?
    public private(set) var isEvent: Bool
    public weak var delegate: InstanceControllerDelegate?

    private let type: CircleType
    private let coords: [Coord]
    private var lastIndex: Int = 0
    private var lock = Threading.Lock()
    private var lastLastCompletedTime: Date?
    private var lastCompletedTime: Date?
    private var currentUuidIndexes: [String: Int]
    private var currentQuestUuidIndexes: [String: [Int]]
    private var currentUuidSeenTime: [String: Date]
    public let useRwForRaid =
        ProcessInfo.processInfo.environment["USE_RW_FOR_RAID"] != nil

    init(name: String, coords: [Coord], type: CircleType, minLevel: UInt8, maxLevel: UInt8,
         accountGroup: String?, isEvent: Bool) {
        self.name = name
        self.minLevel = minLevel
        self.maxLevel = maxLevel
        self.accountGroup = accountGroup
        self.isEvent = isEvent
        self.coords = coords
        self.type = type
        self.lastCompletedTime = Date()
        self.currentUuidIndexes = [:]
        self.currentQuestUuidIndexes = [:]
        self.currentUuidSeenTime = [:]
    }
    
    deinit {
        stop()
    }
    
    func finishedRoute(mysql: MySQL, uuid: String) {
        currentQuestUuidIndexes.removeAll()
        currentUuidIndexes.removeAll()
        currentUuidSeenTime.removeAll()
        lock.unlock()
        delegate?.instanceControllerDone(mysql: mysql, name: name)
    }

    func routeDistance(xcoord: Int, ycoord: Int) -> Int {
        if xcoord < ycoord {
            return ycoord - xcoord
        }
        return ycoord + (coords.count - xcoord)
    }
    
    func checkQuestSpacingDevices(mysql: MySQL, uuid: String) -> Int {
        let currentIndex = currentUuidIndexes[uuid]
        let currentSubRouteFirst = currentQuestUuidIndexes[uuid]?.first ?? 0
        let currentSubRouteLast = currentQuestUuidIndexes[uuid]?.last ?? 0

        if currentQuestUuidIndexes.isEmpty {
            currentQuestUuidIndexes[uuid] = Array(coords.indices)
            Log.debug(message: "[SmartStaticQuest] [\(uuid)] Fresh Start Assigning sub-route \(currentQuestUuidIndexes[uuid]?.first ?? 0) of \(currentQuestUuidIndexes[uuid]?.first ?? 0) - \(currentQuestUuidIndexes[uuid]?.last ?? 0)")

            return currentQuestUuidIndexes[uuid]?.first ?? 0
        }
                
        //Check to see if there are still coords left in this devices subsection
        if let currentSubSection = currentQuestUuidIndexes[uuid], let lastUuidIndex = currentUuidIndexes[uuid] {
            if (currentSubSection.last! > lastUuidIndex) {
                return lastUuidIndex + 1
            } else {
                //Reached the end of the subsection, needs a new section
                currentQuestUuidIndexes[uuid] = []
                currentUuidIndexes[uuid] = coords.count - 1
                Log.debug(message: "[SmartStaticQuest] [\(uuid)] Finished sub-route \(currentIndex) of \(currentSubRouteFirst) - \(currentSubRouteLast)")
            }
        }
        
        //Find the biggest section
        let largestSection = currentQuestUuidIndexes.sorted(by: { return (($0.1.last ?? 1) - (currentUuidIndexes[$0.0] ?? 1)) > (($1.1.last ?? 1) - (currentUuidIndexes[$1.0] ?? 1)) }).first!
        
        if (largestSection.1.count == 0) {
            //We're probably done here
            Log.debug(message: "[SmartStaticQuest] [\(uuid)] Finished instance and sub-route \(currentIndex ?? 99999) of \(currentSubRouteFirst) - \(currentSubRouteLast)")
            finishedRoute(mysql: mysql, uuid: uuid)
            return coords.count - 1
        }
        
        Log.debug(message: "[SmartStaticQuest] [\(uuid)] Largest sub-section: \(largestSection.0) \(largestSection.1.first!) - \(largestSection.1.last!)")
        
        let difference = Double(largestSection.1.last! - currentUuidIndexes[largestSection.0]!) / 2.0
        
        let differenceFloored = Int(floor(difference))
        
        let newMiddle = largestSection.1.last! - differenceFloored
            
//        if coords.count - largestSection.1[newMiddle] < 5 {
//            //New Middle is very close to the end, we're probably done here
//            return coords.count - 1
//        }
            
        //Assign new End to the largest section
        currentQuestUuidIndexes[largestSection.0] = Array(largestSection.1.first!...newMiddle)
            
        //Assign the rest to the new device
        currentQuestUuidIndexes[uuid] = Array(newMiddle...largestSection.1.last!)
        
        Log.debug(message: "[SmartStaticQuest] [\(uuid)] New sub-route \(currentQuestUuidIndexes[uuid]?.first ?? 99999) of \(currentQuestUuidIndexes[uuid]?.first ?? 99999) - \(largestSection.1.last!)")
        
        if (currentIndex == currentQuestUuidIndexes[uuid]?.first && currentSubRouteFirst == newMiddle && currentSubRouteLast == largestSection.1.last!) {
            Log.debug(message: "[SmartStaticQuest] [\(uuid)] Finished instance and sub-route \(currentIndex ?? 99999) of \(currentSubRouteFirst) - \(currentSubRouteLast)")
            finishedRoute(mysql: mysql, uuid: uuid)
            return coords.count - 1
        }
            
        return (currentQuestUuidIndexes[uuid]?.first)!
    }

    func checkSpacingDevices(uuid: String) -> [String: Int] {
        let deadDeviceCutoffTime = Date().addingTimeInterval(-60)
        var liveDevices = [String]()

        // Check if all registered devices are still online and clean list
        for (currentUuid, _) in currentUuidIndexes.sorted(by: { return $0.1 < $1.1 }) {
            let lastSeen = currentUuidSeenTime[currentUuid]
            if lastSeen != nil {
                if lastSeen! < deadDeviceCutoffTime {
                    currentUuidIndexes[currentUuid] = nil
                    currentUuidSeenTime[currentUuid] = nil
                } else {
                    liveDevices.append(currentUuid)
                }
            }
        }

        let nbliveDevices = liveDevices.count
        var distanceToNext = coords.count

        for i in 0..<nbliveDevices {
            if uuid != liveDevices[i] {
                continue
            }
            if i < nbliveDevices - 1 {
                let nextDevice = liveDevices[i+1]
                distanceToNext = routeDistance(xcoord: currentUuidIndexes[uuid]!,
                                               ycoord: currentUuidIndexes[nextDevice]!)
            } else {
                let nextDevice = liveDevices[0]
                distanceToNext = routeDistance(xcoord: currentUuidIndexes[uuid]!,
                                               ycoord: currentUuidIndexes[nextDevice]!)
            }
        }
        return ["numliveDevices": nbliveDevices, "distanceToNext": distanceToNext]
    }

    // swiftlint:disable function_body_length
    func getTask(mysql: MySQL, uuid: String, username: String?, account: Account?) -> [String: Any] {
        var currentIndex = 0
        var currentUuidIndex = 0
        var currentCoord = coords[currentIndex]
        if type == .smartStaticQuest {
            lock.lock()
            
            currentIndex = checkQuestSpacingDevices(mysql: mysql, uuid: uuid)
            currentCoord = coords[currentIndex]
            currentUuidIndexes[uuid] = currentIndex
            lock.unlock()
            let delay: Int
            let encounterTime: UInt32
            do {
                let result = try Cooldown.cooldown(
                    account: account,
                    deviceUUID: uuid,
                    location: currentCoord
                )
                delay = result.delay
                encounterTime = result.encounterTime
            } catch {
                Log.error(message: "[CircleInstanceController] [\(name)] [\(uuid)] Failed to calculate cooldown.")
                return [String: Any]()
            }

            do {
                try Cooldown.encounter(
                    mysql: mysql,
                    account: account,
                    deviceUUID: uuid,
                    location: currentCoord,
                    encounterTime: encounterTime
              )
            } catch {
                Log.error(message: "[CircleInstanceController] [\(name)] [\(uuid)] Failed to store cooldown.")
                return [String: Any]()
            }
            return ["action": "scan_quest", "deploy_egg": false, "lat": currentCoord.lat, "lon": currentCoord.lon, "delay": delay, "min_level": minLevel, "max_level": maxLevel]
        }
        if type == .smartPokemon {
            lock.lock()
            currentUuidIndex = currentUuidIndexes[uuid] ?? Int.random(in: 0..<coords.count)
            currentUuidIndexes[uuid] = currentUuidIndex
            currentUuidSeenTime[uuid] = Date()
            var shouldAdvance = true
            var jumpDistance = 0

            if currentUuidIndexes.count > 1 && Int.random(in: 0...100) < 15 {
                let live = checkSpacingDevices(uuid: uuid)
                let dist = 10 * live["distanceToNext"]! * live["numliveDevices"]! + 5
                if dist < 10 * coords.count {
                    shouldAdvance = false
                }
                if dist > 12 * coords.count {
                    jumpDistance = live["distanceToNext"]! - coords.count / live["numliveDevices"]! - 1
                }
            }
            if currentUuidIndex == 0 {
                shouldAdvance = true
            }
            if shouldAdvance {
                currentUuidIndex += jumpDistance + 1
                if currentUuidIndex >= coords.count - 1 {
                    currentUuidIndex -= coords.count - 1
                    lastLastCompletedTime = lastCompletedTime
                    lastCompletedTime = Date()
                }
            } else {
                currentUuidIndex -= 1
                if currentUuidIndex < 0 {
                    currentUuidIndex = coords.count - 1
                }
            }
            lock.unlock()
            currentUuidIndexes[uuid] = currentUuidIndex
            currentCoord = coords[currentUuidIndex]
            return ["action": "scan_pokemon", "lat": currentCoord.lat, "lon": currentCoord.lon,
                    "min_level": minLevel, "max_level": maxLevel]
        } else {
            lock.lock()
            currentIndex = self.lastIndex
            if lastIndex + 1 == coords.count {
                lastLastCompletedTime = lastCompletedTime
                lastCompletedTime = Date()
                lastIndex = 0
            } else {
                lastIndex += 1
            }
            lock.unlock()
            currentCoord = coords[currentIndex]
            if type == .pokemon {
                return ["action": "scan_pokemon", "lat": currentCoord.lat, "lon": currentCoord.lon,
                        "min_level": minLevel, "max_level": maxLevel]
            } else {
                return ["action": "scan_raid", "lat": currentCoord.lat, "lon": currentCoord.lon,
                        "min_level": minLevel, "max_level": maxLevel]
            }
        }
    }
    // swiftlint:enable function_body_length

    func getStatus(mysql: MySQL, formatted: Bool) -> JSONConvertible? {
        if let lastLast = lastLastCompletedTime, let last = lastCompletedTime {
            let time = Int(last.timeIntervalSince(lastLast))
            if formatted {
                return "Round Time: \(time)s"
            } else {
                return ["round_time": time]
            }
        } else if type == .smartStaticQuest {
            var text = ""
            var completed = 0
            if (currentQuestUuidIndexes.count == 0) {
                return "-"
            }
            var uncompleted = 0
            for (index, subSection) in currentQuestUuidIndexes {
//                if text != "" {
                    text += "<br>"
//                }
                let firstNumber = (subSection.first ?? 0) + 1
                let lastNumber = (subSection.last ?? 0) + 1

                let currentCoordIndex = currentUuidIndexes[index] ?? 1

                let currentPosition = subSection.firstIndex(of: currentCoordIndex) ?? 1

//                let percent = currentPosition / subSection.count

                let percent: Double = Double(currentCoordIndex) / Double(lastNumber)

                text += "\(index): \(Int(percent * 100))% \(firstNumber)...\(currentCoordIndex)...\(lastNumber)"

                uncompleted += (lastNumber - currentCoordIndex)
            }
            let diff = (coords.count - uncompleted)
            let totalCount = self.coords.count
            let completedInt: Double = Double(diff) / Double(totalCount)
            let multiplied = Int(completedInt * 100)
            return "\(multiplied)%\(text)"
        } else {
            if formatted {
                return "-"
            } else {
                return nil
            }
        }
    }

    func reload() {
        lock.lock()
        lastIndex = 0
        lock.unlock()
    }

    func stop() {}

    func getAccount(mysql: MySQL, uuid: String) throws -> Account? {
        switch type {
        case .pokemon, .smartPokemon, .smartStaticQuest:
            return try Account.getNewAccount(
                mysql: mysql,
                minLevel: minLevel,
                maxLevel: maxLevel,
                ignoringWarning: false,
                spins: nil,
                noCooldown: false,
                device: uuid,
                group: accountGroup
            )
        case .raid:
            return try Account.getNewAccount(
                mysql: mysql,
                minLevel: minLevel,
                maxLevel: maxLevel,
                ignoringWarning: useRwForRaid,
                spins: nil,
                noCooldown: false,
                device: uuid,
                group: accountGroup
            )
        }
    }

    func accountValid(account: Account) -> Bool {
        switch type {
        case .pokemon, .smartPokemon, .smartStaticQuest:
            return account.level >= minLevel &&
                account.level <= maxLevel &&
                account.isValid(group: accountGroup)
        case .raid:
            return account.level >= minLevel &&
                account.level <= maxLevel &&
                account.isValid(ignoringWarning: useRwForRaid, group: accountGroup)
        }
    }

}
