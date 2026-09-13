import unreal


MAP = "/Game/RealCitySF/Maps/San_Francisco_sunny_01"
unreal.EditorLevelLibrary.load_level(MAP)
actors = unreal.EditorLevelLibrary.get_all_level_actors()
changed_components = 0
changed_actors = 0

for actor in actors:
    actor_changed = False
    for component in actor.get_components_by_class(unreal.StaticMeshComponent):
        try:
            component.set_collision_enabled(unreal.CollisionEnabled.NO_COLLISION)
            changed_components += 1
            actor_changed = True
        except Exception as error:
            unreal.log_warning("collision update skipped for {}: {}".format(actor.get_name(), error))
    if actor_changed:
        changed_actors += 1

unreal.EditorLevelLibrary.save_current_level()
unreal.log("RealCitySF collision disabled: actors={} components={}".format(changed_actors, changed_components))
