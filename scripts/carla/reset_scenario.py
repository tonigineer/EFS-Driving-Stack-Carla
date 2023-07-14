from planner import CarlaAPI


def main():
    CarlaAPI.remove_actors(pattern=['ego_vehicle'])
    CarlaAPI.spawn_vehicle(blueprint='vehicle.audi.etron',
                           role_name='ego_vehicle')
    CarlaAPI.move_to_actor(pattern=['ego_vehicle'])


if __name__ == "__main__":
    main()
