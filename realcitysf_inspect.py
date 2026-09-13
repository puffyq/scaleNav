import unreal


MAP = "/Game/RealCitySF/Maps/San_Francisco_sunny_01"
unreal.EditorLevelLibrary.load_level(MAP)
actors = unreal.EditorLevelLibrary.get_all_level_actors()
unreal.log("RealCitySF inspect actor_count={}".format(len(actors)))

for actor in actors:
    class_name = actor.get_class().get_name()
    if class_name not in {
        "DirectionalLight",
        "SkyLight",
        "PostProcessVolume",
        "SkyAtmosphere",
        "ExponentialHeightFog",
        "AtmosphericFog",
    }:
        continue
    component = actor.get_component_by_class(unreal.LightComponent)
    details = [
        "class={}".format(class_name),
        "label={}".format(actor.get_actor_label()),
        "hidden={}".format(actor.is_hidden_ed()),
    ]
    if component:
        for name in ("intensity", "mobility", "cast_shadows", "real_time_capture"):
            try:
                details.append("{}={}".format(name, component.get_editor_property(name)))
            except Exception:
                pass
    unreal.log("RealCitySF inspect " + " ".join(details))

unreal.log("RealCitySF inspect done")
