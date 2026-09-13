import unreal

MAP = "/Game/RealCitySF/Maps/San_Francisco_sunny_01"
unreal.EditorLevelLibrary.load_level(MAP)
actors = unreal.EditorLevelLibrary.get_all_level_actors()
unreal.log("collision probe actors={}".format(len(actors)))
seen = 0
for actor in actors:
    components = actor.get_components_by_class(unreal.StaticMeshComponent)
    if not components:
        continue
    component = components[0]
    try:
        collision = component.get_editor_property("collision_enabled")
    except Exception as error:
        collision = "ERROR:{}".format(error)
    unreal.log("collision probe actor={} component={} collision={}".format(actor.get_name(), component.get_name(), collision))
    seen += 1
    if seen >= 10:
        break
unreal.log("collision probe static_mesh_actors={}".format(seen))

