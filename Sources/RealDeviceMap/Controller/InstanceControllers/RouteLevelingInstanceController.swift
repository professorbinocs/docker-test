//
//  RouteLevelingInstanceController.swift
//  RealDeviceMap
//
//  Created by Florian Kostenzer on 29.01.20.
//
//  swiftlint:disable type_body_length function_body_length cyclomatic_complexity

import Foundation
import PerfectLib
import PerfectThread
import PerfectMySQL
import POGOProtos
import Turf
import S2Geometry

class RouteLevelingInstanceController: InstanceControllerProto {
    
    public private(set) var name: String
    public private(set) var minLevel: UInt8
    public private(set) var maxLevel: UInt8
    public private(set) var accountGroup: String?
    public private(set) var isEvent: Bool
    public weak var delegate: InstanceControllerDelegate?
    
    private static let levelXP = [
        0,
        0,
        1000,
        3000,
        6000,
        10000,
        15000,
        21000,
        28000,
        36000,
        45000,
        55000,
        65000,
        75000,
        85000,
        100000,
        120000,
        140000,
        160000,
        185000,
        210000,
        260000,
        335000,
        435000,
        560000,
        710000,
        900000,
        1100000,
        1350000,
        1650000,
        2000000,
        2500000,
        3000000,
        3750000,
        4750000,
        6000000,
        7500000,
        9500000,
        12000000,
        15000000,
        20000000
    ]
    
    enum RouteType {
        case fixed
        case normal
    }
    
    private let start: Coord
    private let storeData: Bool
    private let radius: UInt64
    private let unspunPokestopsPerUsernameLock = NSLock()
    private var unspunPokestopsPerUsername = [String: [Coord]]()
    private var lastPokestopsPerUsername = [String: [Coord]]()
    private let playerLock = NSLock()
    private var playerLastSeen = [String: Date]()
    private var playerXP = [String: Int]()
    private var playerLevel = [String: Int]()
    private var playerXPPerTime = [String: [(date: Date, xp: Int)]]()
    private var lastLocactionUsername = [String: Coord]()
    private var multiPolygon: MultiPolygon
    private var type: RouteType
    private var route: [Coord]
    
    private let stopsLock = Threading.Lock()
    private var allStops = [Coord]()
    private var todaysLevelingStops = [Coord]()
    private var questClearerQueue: ThreadQueue?
    private var shouldExit = false
    private let bootstrappLock = Threading.Lock()
    private var bootstrappCellIDs = [S2CellId]()
    private var bootstrappTotalCount = 0
    private let accountsLock = Threading.Lock()
    private var accounts = [String: String]()
    private var endpoints = [Coord: [Coord]]()
    private var clusterSetPokestops = [Coord: [Coord]]()
    private var clusterSetSpawnpoints = [Coord: [Coord]]()
    
    private var allSpawnpoints = [Coord]()
    
    private var distances = [Coord: [Coord: Double]]()
    
    private var lastAction = [String: Any]()
    
    init(name: String, minLevel: UInt8, maxLevel: UInt8, storeData: Bool, multiPolygon: MultiPolygon, type: RouteType, route: [Coord], start: Coord = Coord(lat: 0.0, lon: 0.0), radius: UInt64 = 0, accountGroup: String?, isEvent: Bool) {
        self.name = name
        self.minLevel = minLevel
        self.maxLevel = maxLevel
        self.storeData = storeData
        self.multiPolygon = multiPolygon
        self.type = type
        self.route = route
        self.start = start
        self.radius = radius
        self.accountGroup = accountGroup
        self.isEvent = isEvent

        update()
    }
    
    private func update() {
        switch type {
        case .normal:
            break
            var routeDistance = 0.0
            stopsLock.lock()
            for polygon in multiPolygon.polygons {
                
                if let bounds = BoundingBox(from: polygon.outerRing.coordinates),
                    let stops = try? Pokestop.getAll(
                        minLat: bounds.southEast.latitude, maxLat: bounds.northWest.latitude,
                        minLon: bounds.northWest.longitude, maxLon: bounds.southEast.longitude,
                        updated: 0, questsOnly: false, showQuests: true, showLures: true, showInvasions: true),
                    let spawnpoints = try? SpawnPoint.getAll(minLat: bounds.southEast.latitude, maxLat: bounds.northWest.latitude,
                                                             minLon: bounds.northWest.longitude, maxLon: bounds.southEast.longitude,
                                                             updated: 0){
                    
                    for stop in stops {
                        let coord = CLLocationCoordinate2D(latitude: stop.lat, longitude: stop.lon)
                        if polygon.contains(coord, ignoreBoundary: false) {
                            print("\(stop.lat),\(stop.lon)")
                            let actualCoord = Coord(lat: stop.lat, lon: stop.lon)
                            self.allStops.append(actualCoord)
                            self.distances[actualCoord] = [:]
                            clusterSetPokestops[actualCoord] = []
                            clusterSetSpawnpoints[actualCoord] = []
                        }
                    }
                    
                    for (index, point) in self.allStops.enumerated() {
                        
                        for spawnpoint in spawnpoints {
                            let coord = CLLocationCoordinate2D(latitude: spawnpoint.lat, longitude: spawnpoint.lon)
                            if polygon.contains(coord, ignoreBoundary: false) {
                                let actualCoord = Coord(lat: spawnpoint.lat, lon: spawnpoint.lon)
                                self.allSpawnpoints.append(actualCoord)
                                
                                let d = distance(A: actualCoord, B: actualCoord)
                                if d <= 40 {
                                    clusterSetSpawnpoints[point]!.append(actualCoord)
                                }
                            }
                        }
                        
                        for (secondIndex, secondPoint) in self.allStops.enumerated() {
                            
                            if index != secondIndex {
                                let d = distance(A: point, B: secondPoint)
                                if d <= 75 {
                                    clusterSetPokestops[point]!.append(secondPoint)
                                }
                            }
                        }
                    }
                }
            }

            var mostNearbySet = [Coord]()
            var leastCoordCount = self.allStops.count
            var minniestNearby = 1
            
            let maxStopNearby = clusterSetPokestops.max { a, b in a.value.count < b.value.count}
                        
            var sortedDiskSet = self.allStops.sorted { self.clusterSetPokestops[$0]!.count > self.clusterSetPokestops[$1]!.count}
            
            while !sortedDiskSet.isEmpty {
                let maxIndex = sortedDiskSet.indices.max { a, b in self.clusterSetPokestops[sortedDiskSet[a]]!.count < self.clusterSetPokestops[sortedDiskSet[b]]!.count }
                let maxDisk = self.clusterSetPokestops[sortedDiskSet[maxIndex!]]
                
                mostNearbySet.append(sortedDiskSet[maxIndex!])

                sortedDiskSet.remove(at: maxIndex!)
                for diskSet in self.clusterSetPokestops {
                    self.clusterSetPokestops[diskSet.key] = diskSet.value.filter { !maxDisk!.contains($0)}
                }
                sortedDiskSet.removeAll( where: {maxDisk!.contains($0)})
                
                sortedDiskSet.removeAll(where: {self.clusterSetPokestops[$0]!.count == 0})
            }
            
            let clusters = generateClusterSet(coords: mostNearbySet)
            
            let oneOff = clusters.map {$0.0}
            
            var arrayOfStopSetsToTSP = [(Int, [Coord])]()
            arrayOfStopSetsToTSP.append((minniestNearby, oneOff))

            var trueShortestSet = self.allStops
            var trueShortestSetLength = tourLengthM(tour: trueShortestSet)
            arrayOfStopSetsToTSP.forEach { stopSet in
                var currentStopSet = stopSet
                var TSPResults = [Coord]()
                TSPResults.append(currentStopSet.1.removeFirst())
                
                routeDistance = 0.0
                
                while !currentStopSet.1.isEmpty {
                    let current = TSPResults.last!
                    var closestDistance: Double = 10000000000000000
                    var closestIndex = 0
                    
                    for (index, stop) in currentStopSet.1.enumerated() {
                        let dist = current.distance(to: stop) / Double(clusterSetPokestops[stop]!.count)
                        if dist < closestDistance {
                            closestDistance = dist
                            closestIndex = index
                        }
                    }
                    routeDistance += closestDistance
                    
                    TSPResults.append(currentStopSet.1[closestIndex])
                    currentStopSet.1.remove(at: closestIndex)
                }
                
                var possibleSolutions = [String: [Coord]]()
                
                Log.debug(message: "[RouteLevelingInstanceController] [\(self.name)] Min-Nearby (\(currentStopSet.0)) Nearby Neighbor Solution: \(Double(round(1000*tourLengthM(tour: TSPResults))/1000))km")
                possibleSolutions["Nearest-Neighbor"] = TSPResults
                
                let greedyTSP = greedy(cities: TSPResults)
                Log.debug(message: "[RouteLevelingInstanceController] [\(self.name)] Min-Nearby (\(currentStopSet.0)) Greedy Solution: \(Double(round(1000*tourLengthM(tour: greedyTSP))/1000))km")
                possibleSolutions["Greedy"] = greedyTSP
                
                let alteredGreedyTSP = alterTour(tour: greedyTSP)
                Log.debug(message: "[RouteLevelingInstanceController] [\(self.name)] Min-Nearby (\(currentStopSet.0)) Altered Greedy Solution: \(Double(round(1000*tourLengthM(tour: alteredGreedyTSP))/1000))km")
                possibleSolutions["Altered Greedy"] = alteredGreedyTSP
                
                var stopCounter = 0
                for solution in possibleSolutions {
                    for (index, stop) in solution.value.enumerated() {
                        var currentSubset = [Coord]()
                        currentSubset.append(stop)
                        
                        var nextStopIndex = index == solution.value.count - 1 ? 0 : index + 1
                        stopCounter = self.clusterSetPokestops[stop]!.count + (self.clusterSetSpawnpoints[stop]?.count ?? 0) / 4
                        
                        while stopCounter < 4000 {
                            let nextStop = solution.value[nextStopIndex]
                            stopCounter += self.clusterSetPokestops[nextStop]!.count + (self.clusterSetSpawnpoints[stop]?.count ?? 0) / 4
                            nextStopIndex = nextStopIndex == solution.value.count - 1 ? 0 : nextStopIndex + 1
                            currentSubset.append(nextStop)
                        }
                        if tourLengthM(tour: currentSubset) < trueShortestSetLength {
                            // Stitch the route back together
                            trueShortestSet = []
                            if index == 0 {
                                trueShortestSet += solution.value[index...]
                            } else if index != solution.value.count - 1 {
                                trueShortestSet += solution.value[index...]
                                trueShortestSet += solution.value[0...index - 1]
                            } else {
                                trueShortestSet.append(solution.value[index])
                                trueShortestSet += solution.value[0...index - 1]
                            }
                            trueShortestSetLength = tourLengthM(tour: currentSubset)
                            
                            Log.debug(message: "[\(solution.key)] [\(self.name)] Min-Nearby (\(currentStopSet.0)) New shortest set found: \(currentSubset.count) Length:  \(trueShortestSetLength)")
                        }
                    }
                }
            }
            
            self.todaysLevelingStops = trueShortestSet
            
            stopsLock.unlock()
        case .fixed:
            Log.debug(message: "[RouteLevelingInstanceController] [\(self.name)] Starting Fixed")

            self.todaysLevelingStops = route
        }
    }

    func greedy(cities: [Coord]) -> [Coord] {
        
        cities.forEach { (city) in
            endpoints[city] = [city]
        }
        let sortedEdges = shortEndpointFirst(cities: cities)
        var newSegment: [Coord] = []
        for (cityA, cityB) in sortedEdges {
            if endpoints[cityA] != nil && endpoints[cityB] != nil && endpoints[cityA] != endpoints[cityB] {
                newSegment = joinEndpoints(cityA: cityA, cityB: cityB)
                if newSegment.count == cities.count { break }
            }
        }
        return newSegment
    }
    
    func shortEndpointFirst(cities: [Coord]) -> [(Coord, Coord)] {
        var edges = [(Coord, Coord)]()
        for (indexA, cityA) in cities.enumerated() {
            for (indexB, cityB) in cities.enumerated() {
                if indexA < indexB {
                    edges.append((cityA, cityB))
                }
            }
        }
        let sortedEdges = edges.sorted {
            let dist1 = distance(A: $0.0, B: $0.1)
            let dist2 = distance(A: $1.0, B: $1.1)

            return dist1 < dist2
        }

        return sortedEdges
    }
    
    func joinEndpoints(cityA: Coord, cityB: Coord) -> [Coord] {
        var segmentA = endpoints.remove(at: endpoints.index(forKey: cityA)!).value
        var segmentB = endpoints.remove(at: endpoints.index(forKey: cityB)!).value
        if segmentA.last != cityA { segmentA = segmentA.reversed() }
        if segmentB.first != cityB { segmentB = segmentB.reversed() }
        segmentA += segmentB

        endpoints[segmentA[0]] = segmentA
        endpoints[segmentA.last!] = segmentA
        return segmentA
    }
    
    func alterTour(tour: [Coord]) -> [Coord] {
        var alteredTour = tour
        let originalLength: Double = tourLength(tour: alteredTour)
        let allSegs = allSegments(N: alteredTour.count)
        for (start, end) in allSegs {
            alteredTour = flippit(tour: alteredTour, i: start, j: end)
        }
        if tourLength(tour: alteredTour) < originalLength {
            return alterTour(tour: alteredTour)
        }
        return tour
    }
    
    func allSegments(N: Int) -> [(Int, Int)] {
        var segs = [(Int, Int)]()
        for length in (2...N).reversed() {
            for start in 0...N-length {
                segs.append((start, start + length))
            }
        }

        return segs
    }
    
    func flippit(tour: [Coord],i: Int,j: Int) -> [Coord] {
        var returnTour = tour
        var indexA = i-1 < 0 ? tour.count - 1 : i-1
        var indexB = i
        var indexC = j-1 < 0 ? tour.count - 1 : j-1
        var indexD = j % tour.count
        let A = tour[indexA]
        let B = tour[i]
        let C = tour[indexC]
        let D = tour[j % tour.count]
        let distanceAB = distance(A: A, B: B)
        let distanceCD = distance(A: C, B: D)
        let distanceAC = distance(A: A, B: C)
        let distanceAD = distance(A: B, B: D)
        if distanceAB + distanceCD > distanceAC + distanceAD {
            if i > j-1 {
                returnTour[j-1...i].reverse()
            } else {
                returnTour[i...j-1].reverse()
            }
            return returnTour
        }
        return tour
    }
    
    func tourLength(tour: [Coord]) -> Double {
        var tourDistance:Double = distance(A: tour.first!, B: tour.last!)
        for i in 0..<tour.count-1 {
            tourDistance += distance(A: tour[i], B: tour[i+1])
        }
        return tourDistance
    }
    
    func tourLengthM(tour: [Coord]) -> Double {
        var tourDistance:Double = distanceTour(A: tour.first!, B: tour.last!)
        for i in 0..<tour.count-1 {
            tourDistance += distanceTour(A: tour[i], B: tour[i+1])
        }
        return tourDistance
    }
    
    func distanceTour(A: Coord, B: Coord) -> Double {
        let R = 6367.0; //Radius of earth in km
        let hPi = 1.5707963;
        let rad = 0.017453292519943
        let lat1 = B.lat * rad
        let lat2 = A.lat * rad
        let lon1 = B.lon * rad
        let lon2 = A.lon * rad
        let a = hPi - lat1;
        let b = hPi - lat2;
        let u = a * a + b * b;
        let v = -2 * a * b * cos(lon2 - lon1);
        let c = sqrt(abs(u + v));
        let result = R * c;
        return result
    }
    
    func distance(A: Coord, B: Coord) -> Double {
        if self.distances[A] == nil {
            self.distances[A] = [:]
        }
        if self.distances[B] == nil {
            self.distances[B] = [:]
        }
        if let d = self.distances[A]![B] ?? self.distances[B]![A] {
            return d
        }
        let R = 6367000.0; //Radius of earth in m
        let hPi = 1.5707963;
        let rad = 0.017453292519943
        let lat1 = B.lat * rad
        let lat2 = A.lat * rad
        let lon1 = B.lon * rad
        let lon2 = A.lon * rad
        let a = hPi - lat1;
        let b = hPi - lat2;
        let u = a * a + b * b;
        let v = -2 * a * b * cos(lon2 - lon1);
        let c = sqrt(abs(u + v));
        let result = R * c // Double(clusterSetPokestops[A]! + clusterSetPokestops[B]!) / 2.0
        self.distances[A]![B] = result
        self.distances[B]![A] = result
        return result
    }
    
    private func generateClusterSet(coords: [Coord]) -> [(Coord, [Coord])] {
        // Greedy sort clusters with generated disks
        
        var disksWithPoints = [Coord: [Coord]]()
        var disksWithSpawnpoints = [Coord: [String]]()
        var startTime = Date()
        // Iterate through our points and generate a set of disks that cover any 2 points
        for point in coords {
            //Create an distance entry for the first point, if it does not exist
            for secondPoint in self.allStops {
                //Create an distance entry for the second point, if it does not exist
                
                if point != secondPoint {
                    let d = distance(A: point, B: secondPoint)
                    if d <= 75 {
                        let disks = generateCircle(a: point, b: secondPoint, r: 75)
                        self.distances[disks[0]] = [:]
                        self.distances[disks[1]] = [:]
                        disksWithPoints[disks[0]] = []
                        
                        disksWithPoints[disks[1]] = []
                    }
                }
            }
        }
        
        for disk in disksWithPoints {
            self.distances[disk.key] = [:]
            for point in coords {
                let d = distance(A: disk.key, B: point)
                if d <= 78.0 {
                    disksWithPoints[disk.key]!.append(point)
                }
            }
        }
        Log.debug(message: "Time for disk creation: \(String(format: "%.3f", Date().timeIntervalSince(startTime)))s)")
        
        startTime = Date()
        var sortedDiskSet = disksWithPoints.sorted { $0.value.count > $1.value.count}
        
        var returnDiskSet = [(Coord, [Coord])]()
        Log.debug(message: "Time for disk sorting: \(String(format: "%.3f", Date().timeIntervalSince(startTime)))s)")
        
        startTime = Date()
        
        while !sortedDiskSet.isEmpty {
            let maxIndex = sortedDiskSet.indices.max { a, b in sortedDiskSet[a].value.count < sortedDiskSet[b].value.count }
            let maxDisk = sortedDiskSet[maxIndex!]

            sortedDiskSet.remove(at: maxIndex!)
            for (index, diskSet) in sortedDiskSet.enumerated() {
                let nearbyCount = sortedDiskSet[index].value.count
                sortedDiskSet[index] = (diskSet.key, diskSet.value.filter { !maxDisk.value.contains($0)})
                let newCount = sortedDiskSet[index].value.count
                if newCount < nearbyCount {
                }
            }
            returnDiskSet.append(maxDisk)
            
            sortedDiskSet.removeAll(where: {$0.value.count == 0})
        }
        Log.debug(message: "Time for disk filtering: \(String(format: "%.3f", Date().timeIntervalSince(startTime)))s)")

        return returnDiskSet
    }
    
    private func pointInsideDisk(center: Coord, r: Double, point: Coord) -> Bool {
        return abs(center.distance(to: point)) <= r
    }
    
    private func generateCircle(a: Coord, b: Coord, r: Double) -> [Coord] {
        let bee = (y: b.lat * (Double.pi / 180), x: b.lon * (Double.pi / 180))
        let ayy = (y: a.lat * (Double.pi / 180), x: a.lon * (Double.pi / 180))
        
        let radius = 0.0000117720922932035787//r //< 41 ? 0.0000117720922932035787 / 2 : 0.0000117720922932035787
        
        let midpointABRads = midPointRad(a: (lat: ayy.y, lon: ayy.x), b: (lat: bee.y, lon: bee.x))
        let midpointAB = (x: midpointABRads.lon, y: midpointABRads.lat)

        let distanceMPA = distanceRad(point1: (lat: ayy.y, lon: ayy.x), point2: (lat: midpointAB.y, lon: midpointAB.x))

        var vectorAB = (x: Double(), y: Double())
        
        vectorAB.x = (bee.x - ayy.x) / sqrt(pow((bee.x - ayy.x), 2) + pow((bee.y - ayy.y), 2))
        
        vectorAB.y = (bee.y - ayy.y) / sqrt(pow((bee.x - ayy.x), 2) + pow((bee.y - ayy.y), 2))
        
        let h = sqrt(pow(radius, 2) - pow(distanceMPA, 2))
        
        var center1 = (x: Double(), y: Double())
        
        center1.x = midpointAB.x + (h * vectorAB.y)
        center1.y = midpointAB.y - (h * vectorAB.x)
        
        var center2 = (x: Double(), y: Double())
        
        center2.x = midpointAB.x - (h * vectorAB.y)
        center2.y = midpointAB.y + (h * vectorAB.x)

        return [Coord(lat:center1.y * 180 / Double.pi, lon: center1.x * 180 / Double.pi), Coord(lat:center2.y * 180 / Double.pi, lon: center2.x * 180 / Double.pi)]
    }
    
    private func midPointRad(a: (lat: Double, lon: Double), b: (lat: Double, lon: Double)) -> (lat: Double, lon: Double) {
        let Bay = (x: cos(b.lat) * cos(b.lon - a.lon), y: cos(b.lat) * sin(b.lon - a.lon))
        
        var mp = (lat: Double(), lon: Double())
        
        mp.lat = atan2(sin(a.lat) + sin(b.lat), sqrt((cos(a.lat) + Bay.x) * (cos(a.lat) + Bay.x) + Bay.y * Bay.y))
        mp.lon = a.lon + atan2(Bay.y, cos(a.lat) + Bay.x)
        
        return mp
    }
    
    private func oldgenerateCircle(a: Coord, b: Coord, r: Double) -> [Coord] {
        let b = (lat: 35.670989 * (Double.pi / 180), lon: 139.748641 * (Double.pi / 180))
        let a = (lat: 35.670837 * (Double.pi / 180), lon: 139.748365 * (Double.pi / 180))

        let distanceAB = distanceRad(point1: a, point2: b)

        let radRadius = 0.00001174740865641924 //75.0 / 6371000.0 //0.00001174740865641924
        
        let distanceAR = radRadius //degreeToDistanceRad(point1: Coord(lat: 35.710142, lon: 139.773895), point2: rCoord)
        
        let angleRad = acos((distanceAB / 2) / distanceAR)
        
        let angleLat = asin(sin(b.lat)*cos(distanceAR)+cos(b.lat)*sin(distanceAR)*cos(angleRad))
        
        let angleLon = (b.lon-asin(sin(angleRad)*sin(distanceAR)/cos(b.lat))+Double.pi).truncatingRemainder(dividingBy: 2 * Double.pi) - Double.pi

        let angleCoord = (lat: angleLat, lon: angleLon)
        
        Log.debug(message: "Point A: \(a.lat * 180 / Double.pi), \(a.lon * 180 / Double.pi) Point B: \(b.lat * 180 / Double.pi), \(b.lon * 180 / Double.pi) Circle 1: \(angleCoord.lat * 180 / Double.pi), \(angleCoord.lon * 180 / Double.pi)")
        Log.debug(message: "distanceAB: \(distanceAB), angleRaid: \(angleRad)")
        
        let lonAbsDiff = abs(b.lon - angleCoord.lon)
        let latAbsDiff = abs(b.lat - angleCoord.lat)
        
        let otherAbsLon = a.lon + lonAbsDiff
        let otherAbsLat = a.lat - latAbsDiff
        
        let otherReverseAbsLon = b.lon + lonAbsDiff
        let otherReverseAbsLat = b.lat + latAbsDiff
        
        Log.debug(message: "Abs Pt: \(otherAbsLat * 180 / Double.pi), \(otherAbsLon * 180 / Double.pi)")
        
        Log.debug(message: "Reverse Abs Pt: \(otherReverseAbsLat * 180 / Double.pi), \(otherReverseAbsLon * 180 / Double.pi)")

        let otherLon = otherAbsLon
        let otherLat = otherAbsLat
        
        let angleBCoord = (lat: otherLat, lon: otherLon)
        
        let angleBRad = angleRad
        
        let distanceBAR = distanceAR
        
        let angleBLat = asin(sin(b.lat)*cos(distanceBAR)-cos(b.lat)*sin(distanceBAR)*cos(angleBRad))
        
        let angleBLon = (b.lon-asin(sin(angleBLat)*sin(distanceBAR)/cos(b.lat))+Double.pi).truncatingRemainder(dividingBy: 2 * Double.pi) - Double.pi

        Log.debug(message: "Point A: \(a.lat * 180 / Double.pi), \(a.lon * 180 / Double.pi) Point B: \(b.lat * 180 / Double.pi), \(b.lon * 180 / Double.pi) Circle 2: \(angleBCoord.lat * 180 / Double.pi), \(angleBCoord.lon * 180 / Double.pi)")
        Log.debug(message: "distanceAB: \(distanceAB), angleRaid: \(angleRad)")
        Log.debug(message: "NEW Distance from Point A to Circle1: \(distanceRadM(point1: a, point2: angleCoord))")
        Log.debug(message: "NEW Distance from Point B to Circle1: \(distanceRadM(point1: b, point2: angleCoord))")
        Log.debug(message: "NEW Distance from Point A to Circle2: \(distanceRadM(point1: a, point2: angleBCoord))")
        Log.debug(message: "NEW Distance from Point B to Circle2: \(distanceRadM(point1: b, point2: angleBCoord))")
        Log.debug(message: "Done")

        return [Coord(lat: a.lat, lon: a.lon)]
    }
    
    func degreeToDistanceM(point1: Coord, point2: Coord) -> Double {
        let lat1Rad = point1.lat * Double.pi / 180.0
        let lon1Rad = point1.lon * Double.pi / 180.0
        
        let lat2Rad = point2.lat * Double.pi / 180.0
        let lon2Rad = point2.lon * Double.pi / 180.0

        let part1 = sin(lat1Rad)*sin(lat2Rad)
        let part2 = cos(lat1Rad)*cos(lat2Rad)*cos(lon1Rad-lon2Rad)
        let d = acos(part1 + part2)
        
        let firstPow = pow((sin((lat1Rad-lat2Rad)/2)), 2)
        let secondPow = pow((sin((lon1Rad-lon2Rad)/2)), 2)
        
        let distance = 2*asin(sqrt(firstPow + cos(lat1Rad)*cos(lat2Rad)*secondPow))
        
        let distanceOther = distance * 6371000
        
        let distanceMeters = d * 6371000

        return distanceOther
    }
    
    func distanceRad(point1: (lat: Double, lon: Double), point2: (lat: Double, lon: Double)) -> Double {
        let lat1Rad = point1.lat
        let lon1Rad = point1.lon
        
        let lat2Rad = point2.lat
        let lon2Rad = point2.lon

        let part1 = sin(lat1Rad)*sin(lat2Rad)
        let part2 = cos(lat1Rad)*cos(lat2Rad)*cos(lon1Rad-lon2Rad)
        let distance = acos(part1 + part2)
        
        let firstPow = pow((sin((lat1Rad-lat2Rad)/2)), 2)
        let secondPow = pow((sin((lon1Rad-lon2Rad)/2)), 2)
        
        let d = 2*asin(sqrt(firstPow + cos(lat1Rad)*cos(lat2Rad)*secondPow))
        
        let distanceMeters = distance
        
        return distanceMeters
    }
    
    func distanceRadM(point1: (lat: Double, lon: Double), point2: (lat: Double, lon: Double)) -> Double {
        let lat1Rad = point1.lat
        let lon1Rad = point1.lon
        
        let lat2Rad = point2.lat
        let lon2Rad = point2.lon

        let part1 = sin(lat1Rad)*sin(lat2Rad)
        let part2 = cos(lat1Rad)*cos(lat2Rad)*cos(lon1Rad-lon2Rad)
        let distance = acos(part1 + part2)
        
        let firstPow = pow((sin((lat1Rad-lat2Rad)/2)), 2)
        let secondPow = pow((sin((lon1Rad-lon2Rad)/2)), 2)
        
        let d = 2*asin(sqrt(firstPow + cos(lat1Rad)*cos(lat2Rad)*secondPow))
        
        let distanceMeters = distance * 6371000
        
        return distanceMeters
    }
    
    func degreeToDistanceRad(point1: Coord, point2: Coord) -> Double {
        let lat1Rad = point1.lat * Double.pi / 180.0
        let lon1Rad = point1.lon * Double.pi / 180.0
        
        let lat2Rad = point2.lat * Double.pi / 180.0
        let lon2Rad = point2.lon * Double.pi / 180.0

        let part1 = sin(lat1Rad)*sin(lat2Rad)
        let part2 = cos(lat1Rad)*cos(lat2Rad)*cos(lon1Rad-lon2Rad)
        let distance = acos(part1 + part2)
        
        let firstPow = pow((sin((lat1Rad-lat2Rad)/2)), 2)
        let secondPow = pow((sin((lon1Rad-lon2Rad)/2)), 2)
        
        let d = 2*asin(sqrt(firstPow + cos(lat1Rad)*cos(lat2Rad)*secondPow))
        
        let distanceMeters = distance
        
        return distanceMeters
    }
    
    func getTask(mysql: MySQL, uuid: String, username: String?, account: Account?) -> [String: Any] {
        
        guard let username = username else {
            Log.error(message: "[RouteLevelingInstanceController] [\(name)] [\(uuid)] No username specified.")
            return [:]
        }
        
        guard let account = account else {
            Log.error(message: "[RouteLevelingInstanceController] [\(name)] [\(uuid)] No account specified.")
            return [:]
        }
        
        unspunPokestopsPerUsernameLock.lock()
        
        if lastPokestopsPerUsername[username] == nil {
            lastPokestopsPerUsername[username] = []
        }
        if unspunPokestopsPerUsername[username] == nil {
            unspunPokestopsPerUsername[username] = todaysLevelingStops
        }
        let destination: Coord

        if let lastA = self.lastAction["action"] as? String,
            let lastLat = self.lastAction["lat"] as? Double,
            let lastLon = self.lastAction["lon"] as? Double,
            let minLevel = self.lastAction["min_level"] as? UInt8,
            let maxLevel = self.lastAction["max_level"] as? UInt8 {
            if lastA == "scan_pokemon" {
                let action = [
                    "action": "spin_pokestop",
                    "deploy_egg": true,
                    "lat": lastLat,
                    "lon": lastLon,
                    "delay": 0,
                    "min_level": minLevel,
                    "max_level": maxLevel
                    ] as [String : Any]
                self.lastAction = action
                Log.debug(message: "\(unspunPokestopsPerUsername[username]!.count) remaining stops")
                unspunPokestopsPerUsernameLock.unlock()
                return action
            } else if lastA == "spin_pokestop" {
                if let us = unspunPokestopsPerUsername[username] {
                    if !us.isEmpty {
                        destination = unspunPokestopsPerUsername[username]!.removeFirst()
                    } else {
                        unspunPokestopsPerUsername[username] = todaysLevelingStops
                        destination = unspunPokestopsPerUsername[username]!.removeFirst()
                    }
                } else {
                    unspunPokestopsPerUsername[username] = todaysLevelingStops
                    destination = unspunPokestopsPerUsername[username]!.removeFirst()
                }
                let action = ["action": "scan_pokemon",
                              "lat": destination.lat,
                              "lon": destination.lon,
                              "min_level": minLevel,
                              "max_level": maxLevel] as [String : Any]
                self.lastAction = action
                unspunPokestopsPerUsernameLock.unlock()
                return action
            }
        }
        
        if let us = unspunPokestopsPerUsername[username] {
            if !us.isEmpty {
                destination = unspunPokestopsPerUsername[username]!.removeFirst()
            } else {
                unspunPokestopsPerUsername[username] = todaysLevelingStops
                destination = unspunPokestopsPerUsername[username]!.removeFirst()
            }
        } else {
            unspunPokestopsPerUsername[username] = todaysLevelingStops
            destination = unspunPokestopsPerUsername[username]!.removeFirst()
        }
        
        unspunPokestopsPerUsernameLock.unlock()
        
        let delay: Int
        let encounterTime: UInt32
        do {
            let result = try Cooldown.cooldown(
                account: account,
                deviceUUID: uuid,
                location: destination
            )
            delay = result.delay
            encounterTime = result.encounterTime
        } catch {
            Log.error(message: "[RouteLevelingInstanceController] [\(name)] [\(uuid)] Failed to calculate cooldown.")
            return [String: Any]()
        }
        
        do {
            try Cooldown.encounter(
                mysql: mysql,
                account: account,
                deviceUUID: uuid,
                location: destination,
                encounterTime: encounterTime
            )
        } catch {
            Log.error(message: "[RouteLevelingInstanceController] [\(name)] [\(uuid)] Failed to store cooldown.")
            return [String: Any]()
        }
        
        playerLock.lock()
        playerLastSeen[username] = Date()
        playerLock.unlock()
        
        let action = ["action": "scan_pokemon", "lat": destination.lat, "lon": destination.lon,
                      "min_level": minLevel, "max_level": maxLevel] as [String: Any]
        
        self.lastAction = action
        
        Log.debug(message: action.description)
        
        return action
    }
    
    private func findClosest(unspunPokestops: [String: Coord], exclude: [Coord], username: String,
                             account: Account) -> (key: String, value: Coord)? {
        var closest: (key: String, value: Coord)?
        var closestDistance: Double = 10000000000000000
        
        let current = Coord(
            lat: account.lastEncounterLat ?? start.lat,
            lon: account.lastEncounterLon ?? start.lon
        )
        unspunLoop: for stop in unspunPokestops {
            let coord = Coord(lat: stop.value.lat, lon: stop.value.lon)
            for last in exclude {
                if coord.distance(to: last) <= 40 {
                    continue unspunLoop
                }
            }
            
            let dist = current.distance(to: coord)
            if dist < closestDistance {
                closest = stop
                closestDistance = dist
            }
        }
        return closest
    }
    
    func gotFortData(fortData: PokemonFortProto, username: String?) {
        guard let username = username else {
            return
        }
    }
    
    func gotPlayerInfo(username: String, level: Int, xp: Int) {
        playerLock.lock()
        playerLevel[username] = level
        playerXP[username] = xp
        if playerXPPerTime[username] == nil {
            playerXPPerTime[username] = []
        }
        playerXPPerTime[username]!.append((Date(), xp))
        playerLock.unlock()
    }
    
    func getStatus(mysql: MySQL, formatted: Bool) -> JSONConvertible? {
        var players = [String]()
        playerLock.lock()
        for player in playerLastSeen {
            if Date().timeIntervalSince(player.value) <= 3600 {
                players.append(player.key)
            }
        }
        
        var data = [[String: Any]]()
        for player in players {
            let xpTarget = RouteLevelingInstanceController.levelXP[min(max(Int(maxLevel) + 1, 0), 40)]
            let xpStart = RouteLevelingInstanceController.levelXP[min(max(Int(minLevel), 0), 40)]
            let xpCurrent = playerXP[player] ?? 0
            let xpPercentage = Double(xpCurrent - xpStart) / Double(xpTarget - xpStart) * 100
            var startXP = 0
            var startTime = Date()
            
            for xpPerTime in playerXPPerTime[player] ?? [] {
                if Date().timeIntervalSince(xpPerTime.date) <= 3600 {
                    startXP = xpPerTime.xp
                    startTime = xpPerTime.date
                    break
                } else {
                    _ = playerXPPerTime.popFirst()
                }
            }
            
            let xpDelta = (playerXPPerTime[player]?.last?.xp ?? startXP) - startXP
            let timeDelta = max(
                1,
                (playerXPPerTime[player]?.last?.date.timeIntervalSince1970 ?? startTime.timeIntervalSince1970) -
                    startTime.timeIntervalSince1970
            )
            let xpPerHour = Int(Double(xpDelta) / timeDelta * 3600)
            let timeLeft = Double(xpTarget - xpCurrent) / Double(xpPerHour)
            
            data.append([
                "xp_target": xpTarget,
                "xp_start": xpStart,
                "xp_current": xpCurrent,
                "xp_percentage": xpPercentage,
                "level": playerLevel[player] ?? 0,
                "username": player,
                "xp_per_hour": xpPerHour,
                "time_left": timeLeft
            ])
        }
        
        if formatted {
            var text = ""
            for player in data {
                if text != "" {
                    text += "<br>"
                }
                let username = player["username"] as? String ?? "?"
                let level = player["level"] as? Int ?? 0
                let xpPercentage = (player["xp_percentage"] as? Double ?? 0).rounded(decimals: 1)
                let xpPerHour = (player["xp_per_hour"] as? Int ?? 0).withCommas()
                let timeLeft = player["time_left"] as? Double ?? 0
                
                let timeLeftHours: Int
                let timeLeftMinutes: Int
                if timeLeft == .infinity || timeLeft == -.infinity || timeLeft.isNaN {
                    timeLeftHours = 999
                    timeLeftMinutes = 0
                } else {
                    timeLeftHours = Int(timeLeft)
                    timeLeftMinutes = Int((timeLeft - Double(timeLeftHours)) * 60)
                }
                
                if level > maxLevel {
                    text += "\(username): Lvl.\(level) done"
                } else {
                    text += "\(username): Lvl.\(level) \((xpPercentage))% \(xpPerHour)XP/h " +
                    "\(timeLeftHours)h\(timeLeftMinutes)m"
                }
            }
            playerLock.unlock()
            if text == "" {
                text = "-"
            }
            return text
        } else {
            playerLock.unlock()
            return data
        }
    }
    
    func shouldStoreData() -> Bool {
        return storeData
    }
    
    func reload() {
        
    }
    
    func stop() {
        
    }
    
    func getAccount(mysql: MySQL, uuid: String) throws -> Account? {
        return try Account.getNewAccount(
            mysql: mysql,
            minLevel: minLevel,
            maxLevel: maxLevel,
            ignoringWarning: false,
            spins: nil, // 7000
            noCooldown: true,
            device: uuid,
            group: accountGroup
        )
    }
    
    func accountValid(account: Account) -> Bool {
        return
            account.level >= minLevel &&
                account.level <= maxLevel &&
                account.isValid(group: accountGroup) // && account.hasSpinsLeft(spins: 7000)
    }
    
}
