//
//  IVInstanceController.swift
//  RealDeviceMap
//
//  Created by Florian Kostenzer on 06.11.18.
//

import Foundation
import PerfectLib
import PerfectThread
import PerfectMySQL
import Turf
import S2Geometry

class IVInstanceController: InstanceControllerProto {

    public private(set) var name: String
    public private(set) var minLevel: UInt8
    public private(set) var maxLevel: UInt8
    public private(set) var accountGroup: String?
    public private(set) var isEvent: Bool
    public private(set) var scatterPokemon: [UInt16]

    public weak var delegate: InstanceControllerDelegate?

    private var multiPolygon: MultiPolygon
    private var pokemonList: [UInt16]
    private var pokemonQueue = [Pokemon]()
    private var pokemonLock = Threading.Lock()
    private var scannedPokemon = [(Date, Pokemon)]()
    private var scannedPokemonLock = Threading.Lock()
    private var checkScannedThreadingQueue: ThreadQueue?
    private var staleCellThreadingQueue: ThreadQueue?
    private var statsLock = Threading.Lock()
    private var startDate: Date?
    private var count: UInt64 = 0
    private var shouldExit = false
    private var ivQueueLimit = 100
    private var crappyCenter: CLLocationCoordinate2D
    private let staleLock = Threading.Lock()
    private var staleCellIDs = [CLLocationCoordinate2D]()
    private var staleTotalCount = 0
    private var instanceCellIDs = [UInt64]()

    // swiftlint:disable:next function_body_length
    init(name: String, multiPolygon: MultiPolygon, pokemonList: [UInt16], minLevel: UInt8,
         maxLevel: UInt8, ivQueueLimit: Int, scatterPokemon: [UInt16], accountGroup: String?,
         isEvent: Bool) {
        self.name = name
        self.minLevel = minLevel
        self.maxLevel = maxLevel
        self.accountGroup = accountGroup
        self.isEvent = isEvent
        self.multiPolygon = multiPolygon
        self.pokemonList = pokemonList
        self.ivQueueLimit = ivQueueLimit
        self.scatterPokemon = scatterPokemon
        self.crappyCenter = CLLocationCoordinate2D(latitude: multiPolygon.coordinates.first?.first?.first?.latitude ?? 5.0, longitude: multiPolygon.coordinates.first?.first?.first?.longitude ?? 5.0)

        try? bootstrap()
        
        staleCellThreadingQueue = Threading.getQueue(name: "\(name)-check-stale", type: .serial)
        staleCellThreadingQueue!.dispatch {

            while !self.shouldExit {
                Threading.sleep(seconds: 5.0 * 60.0)
                if self.shouldExit {
                    return
                }
                try? self.checkStaleCells()
            }
        }
        
        checkScannedThreadingQueue = Threading.getQueue(name: "\(name)-check-scanned", type: .serial)
        checkScannedThreadingQueue!.dispatch {

            while !self.shouldExit {

                self.scannedPokemonLock.lock()
                if self.scannedPokemon.isEmpty {
                    self.scannedPokemonLock.unlock()
                    Threading.sleep(seconds: 5.0)
                    if self.shouldExit {
                        return
                    }
                } else {
                    let first = self.scannedPokemon.removeFirst()
                    self.scannedPokemonLock.unlock()
                    let timeSince = Date().timeIntervalSince(first.0)
                    if timeSince < 120 {
                        Threading.sleep(seconds: 120 - timeSince)
                        if self.shouldExit {
                            return
                        }
                    }
                    var success = false
                    var pokemonReal: Pokemon?
                    while !success {
                        do {
                            pokemonReal = try Pokemon.getWithId(id: first.1.id, isEvent: first.1.isEvent)
                            success = true
                        } catch {
                            Threading.sleep(seconds: 1.0)
                            if self.shouldExit {
                                return
                            }
                        }
                    }
                    if let pokemonReal = pokemonReal {
                        if pokemonReal.atkIv == nil {
                            Log.debug(message: "[IVInstanceController] [\(name)] Checked Pokemon doesn't have IV")
                            self.gotPokemon(pokemon: pokemonReal)
                        } else {
                            Log.debug(message: "[IVInstanceController] [\(name)] Checked Pokemon has IV")
                        }
                    }

                }

            }

        }
    }
    
    private func bootstrap() throws {
        Log.info(message: "[IVInstanceController] [\(name)] Checking Bootstrap Status...")
        let start = Date()
        var totalCount = 0
        var missingCellIDs = [S2CellId]()
        var allCellIDs = [UInt64]()
        for polygon in multiPolygon.polygons {
            var sumLat: Double = 0.0
            var sumLon: Double = 0.0
            var sumN: Double = 0.0
            for poly in polygon.coordinates {
                for point in poly {
                    sumLat += point.latitude
                    sumLon += point.longitude
                    sumN += 1.0
                }
            }
            crappyCenter = CLLocationCoordinate2D(latitude: sumLat / sumN, longitude: sumLon / sumN)
            
            let cellIDs = polygon.getS2CellIDs(minLevel: 15, maxLevel: 15, maxCells: Int.max)
            totalCount += cellIDs.count
            let ids = cellIDs.map({ (id) -> UInt64 in
                return id.uid
            })
            allCellIDs.append(contentsOf: ids)
            let cells = try Cell.getInIDs(ids: ids).sorted(by: { (cellA, cellB) -> Bool in
                return cellA.updated ?? 0 > cellB.updated ?? 0
            })

            let parentCellIDs = polygon.getNonInternalS2CellIDs(minLevel: 13, maxLevel: 13, maxCells: Int.max)
            let parentCells = parentCellIDs.map({ (id) -> S2Cell in
                return S2Cell(cellId: id)
            })
            
            let existingCellIds = cells.map { (c) -> (S2CellId, UInt32?) in
                return (S2CellId(id: Int64(c.id)), c.updated)
            }
            
            for cell in existingCellIds {
                let secondsSinceUpdate = Date().timeIntervalSince1970 - Double(cell.1 ?? 0)
                if secondsSinceUpdate > 60 * 5 {
                    if let pC = parentCells.first(where: {$0.contains(cell: S2Cell(cellId: cell.0))}) {
                        if !missingCellIDs.contains(pC.cellId) {
                            missingCellIDs.append(pC.cellId)
                        }
                    } else {
                        if !missingCellIDs.contains(where: {$0 == cell.0 || S2LatLng(point: S2Cell(cellId: $0).center).getDistance(to: S2LatLng(point: S2Cell(cellId: cell.0).center), radius: S2LatLng.earthRadiusMeters) <= 600}) {
                            missingCellIDs.append(cell.0)
                        }
                    }
                }
            }
        }

        staleLock.lock()
        instanceCellIDs = allCellIDs
        staleCellIDs = missingCellIDs.map({ (c) -> CLLocationCoordinate2D in
            let cell = S2Cell(cellId: c)
            let center = S2LatLng(point: cell.center)
            let coord = center.coord
            return CLLocationCoordinate2D(latitude: coord.latitude, longitude: coord.longitude)
        })
        staleTotalCount = missingCellIDs.count
        Log.info(message: "[IVInstanceController] [\(name)] Found \(staleCellIDs.count) stale cells")
        staleLock.unlock()

    }
    
    private func checkStaleCells() throws {
        Log.info(message: "[IVInstanceController] [\(name)] Checking for stale cells")
        let start = Date()
        var totalCount = 0
        var missingCellIDs = [S2CellId]()
        var allCellIDs = [UInt64]()
        for polygon in multiPolygon.polygons {
            var sumLat: Double = 0.0
            var sumLon: Double = 0.0
            var sumN: Double = 0.0
            for poly in polygon.coordinates {
                for point in poly {
                    sumLat += point.latitude
                    sumLon += point.longitude
                    sumN += 1.0
                }
            }
            crappyCenter = CLLocationCoordinate2D(latitude: sumLat / sumN, longitude: sumLon / sumN)
            
            let cellIDs = polygon.getS2CellIDs(minLevel: 15, maxLevel: 15, maxCells: Int.max)
            totalCount += cellIDs.count
            let ids = cellIDs.map({ (id) -> UInt64 in
                return id.uid
            })
            allCellIDs.append(contentsOf: ids)
            let cells = try Cell.getInIDs(ids: ids).sorted(by: { (cellA, cellB) -> Bool in
                return cellA.updated ?? 0 > cellB.updated ?? 0
            })

            let parentCellIDs = polygon.getNonInternalS2CellIDs(minLevel: 13, maxLevel: 13, maxCells: Int.max)
            let parentCells = parentCellIDs.map({ (id) -> S2Cell in
                return S2Cell(cellId: id)
            })
            
            let existingCellIds = cells.map { (c) -> (S2CellId, UInt32?) in
                return (S2CellId(id: Int64(c.id)), c.updated)
            }
            
            for cell in existingCellIds {
                let secondsSinceUpdate = Date().timeIntervalSince1970 - Double(cell.1 ?? 0)
                if secondsSinceUpdate > 60 * 5 {
                    if let pC = parentCells.first(where: {$0.contains(cell: S2Cell(cellId: cell.0))}) {
                        if !missingCellIDs.contains(pC.cellId) {
                            missingCellIDs.append(pC.cellId)
                        }
                    } else {
                        if !missingCellIDs.contains(where: {$0 == cell.0 || S2LatLng(point: S2Cell(cellId: $0).center).getDistance(to: S2LatLng(point: S2Cell(cellId: cell.0).center), radius: S2LatLng.earthRadiusMeters) <= 600}) {
                            missingCellIDs.append(cell.0)
                        }
                    }
                }
            }
        }

        staleLock.lock()
        instanceCellIDs = allCellIDs
        staleCellIDs = missingCellIDs.map({ (c) -> CLLocationCoordinate2D in
            let cell = S2Cell(cellId: c)
            let center = S2LatLng(point: cell.center)
            let coord = center.coord
            return CLLocationCoordinate2D(latitude: coord.latitude, longitude: coord.longitude)
        })
        staleTotalCount = missingCellIDs.count
        Log.info(message: "[IVInstanceController] [\(name)] Found \(staleCellIDs.count) stale cells")
        staleLock.unlock()
    }

    deinit {
        stop()
    }

    func getTask(mysql: MySQL, uuid: String, username: String?, account: Account?) -> [String: Any] {

        staleLock.lock()
        if !staleCellIDs.isEmpty {
            let nextStaleCell = staleCellIDs.popLast()
            staleLock.unlock()
            Log.info(message: "[IVInstanceController] [\(name)] Sending worker to stale cell, \(staleCellIDs.count) stale cells left")
            return ["action": "scan_pokemon", "lat": nextStaleCell!.latitude, "lon": nextStaleCell!.longitude,
            "min_level": minLevel, "max_level": maxLevel]
        }
        staleLock.unlock()
        pokemonLock.lock()
        if pokemonQueue.isEmpty {
            pokemonLock.unlock()
            return ["action": "scan_pokemon", "lat": self.crappyCenter.latitude, "lon": self.crappyCenter.longitude,
            "min_level": minLevel, "max_level": maxLevel]
        }
        let pokemon = pokemonQueue.removeFirst()
        pokemonLock.unlock()

        if UInt32(Date().timeIntervalSince1970) - (pokemon.firstSeenTimestamp ?? 1) >= 600 {
            return getTask(mysql: mysql, uuid: uuid, username: username, account: account)
        }

        scannedPokemonLock.lock()
        scannedPokemon.append((Date(), pokemon))
        scannedPokemonLock.unlock()

        return ["action": "scan_iv", "lat": pokemon.lat, "lon": pokemon.lon, "id": pokemon.id,
                "is_spawnpoint": pokemon.spawnId != nil, "min_level": minLevel, "max_level": maxLevel]
    }

    func getStatus(mysql: MySQL, formatted: Bool) -> JSONConvertible? {

        let ivh: Int?
        self.statsLock.lock()
        if self.startDate != nil {
            ivh = Int(Double(self.count) / Date().timeIntervalSince(self.startDate!) * 3600)
        } else {
            ivh = nil
        }
        self.statsLock.unlock()
        if formatted {
            let ivhString: String
            if ivh == nil {
                ivhString = "-"
            } else {
                ivhString = "\(ivh!)"
            }
            return "<a href=\"/dashboard/instance/ivqueue/\(name.encodeUrl() ?? "")\">Queue" +
                   "</a>: \(pokemonQueue.count), IV/h: \(ivhString)"
        } else {
            return ["iv_per_hour": ivh]
        }
    }

    func reload() {}

    func stop() {
        self.shouldExit = true
        if checkScannedThreadingQueue != nil {
            Threading.destroyQueue(checkScannedThreadingQueue!)
        }
        if staleCellThreadingQueue != nil {
            Threading.destroyQueue(staleCellThreadingQueue!)
        }
        
    }

    func getQueue() -> [Pokemon] {
        pokemonLock.lock()
        let pokemon = self.pokemonQueue
        pokemonLock.unlock()
        return pokemon
    }

    func gotPokemon(pokemon: Pokemon) {
        if (pokemon.pokestopId != nil || pokemon.spawnId != nil) &&
           pokemon.isEvent == isEvent &&
           pokemonList.contains(pokemon.pokemonId) &&
           multiPolygon.contains(CLLocationCoordinate2D(latitude: pokemon.lat, longitude: pokemon.lon)) {
            pokemonLock.lock()

            if pokemonQueue.contains(pokemon) {
                pokemonLock.unlock()
                return
            }

            let index = lastIndexOf(pokemonId: pokemon.pokemonId)

            if pokemonQueue.count >= ivQueueLimit && index == nil {
                Log.debug(message: "[IVInstanceController] [\(name)] Queue is full!")
            } else if pokemonQueue.count >= ivQueueLimit {
                pokemonQueue.insert(pokemon, at: index!)
                _ = pokemonQueue.popLast()
            } else if index != nil {
                pokemonQueue.insert(pokemon, at: index!)
            } else {
                pokemonQueue.append(pokemon)
            }
            pokemonLock.unlock()
        }

    }

    func gotIV(pokemon: Pokemon) {

        if multiPolygon.contains(CLLocationCoordinate2D(latitude: pokemon.lat, longitude: pokemon.lon)) {

            pokemonLock.lock()
            if let index = pokemonQueue.firstIndex(of: pokemon) {
                pokemonQueue.remove(at: index)
            }
            // Re-Scan 100% none event Pokemon
            if isEvent && !pokemon.isEvent && (
                pokemon.atkIv == 15 || pokemon.atkIv == 0 || pokemon.atkIv == 1
               ) && pokemon.defIv == 15 && pokemon.staIv == 15 {
                pokemon.isEvent = true
                pokemonQueue.insert(pokemon, at: 0)
            }
            pokemonLock.unlock()

            self.statsLock.lock()
            if self.startDate == nil {
                self.startDate = Date()
            }
            if self.count == UInt64.max {
                self.count = 0
                self.startDate = Date()
            } else {
                self.count += 1
            }
            self.statsLock.unlock()

        }
    }

    private func lastIndexOf(pokemonId: UInt16) -> Int? {

        guard let targetPriority = pokemonList.firstIndex(of: pokemonId) else {
            return nil
        }

        var i = 0
        for pokemon in pokemonQueue {
            if let priority = pokemonList.firstIndex(of: pokemon.pokemonId), targetPriority < priority {
                return i
            }
            i += 1
        }

        return nil

    }

    func getAccount(mysql: MySQL, uuid: String) throws -> Account? {
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
    }

    func accountValid(account: Account) -> Bool {
        return
            account.level >= minLevel &&
            account.level <= maxLevel &&
            account.isValid(group: accountGroup)
    }

}
