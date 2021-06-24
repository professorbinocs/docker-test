ALTER TABLE instance
MODIFY `type` enum('circle_pokemon','circle_smart_pokemon','circle_raid','circle_smart_raid','auto_quest','pokemon_iv','leveling','route_leveling','fixed_route_leveling') NOT NULL;
