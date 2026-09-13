import unreal


MAP = "/Game/RealCitySF/Maps/San_Francisco_sunny_01"


def actor_by_label_or_class(actors, class_name):
    for actor in actors:
        if actor.get_class().get_name() == class_name:
            return actor
    return None


def actor_by_label(actors, label):
    for actor in actors:
        if actor.get_actor_label() == label:
            return actor
    return None


unreal.EditorLevelLibrary.load_level(MAP)
actors = unreal.EditorLevelLibrary.get_all_level_actors()

directional = actor_by_label_or_class(actors, "DirectionalLight")
if directional is None:
    directional = unreal.EditorLevelLibrary.spawn_actor_from_class(
        unreal.DirectionalLight, unreal.Vector(0, 0, 1000), unreal.Rotator(-45, -35, 0))
directional.set_actor_rotation(unreal.Rotator(-45, -35, 0), False)
directional.set_actor_label("RealCitySF_Daylight_Sun")
directional_component = directional.get_component_by_class(unreal.DirectionalLightComponent)
directional_component.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
directional_component.set_editor_property("intensity", 10.0)

fill = actor_by_label(actors, "RealCitySF_Daylight_Fill")
if fill is None:
    fill = unreal.EditorLevelLibrary.spawn_actor_from_class(
        unreal.DirectionalLight, unreal.Vector(0, 0, 1100), unreal.Rotator(-35, 145, 0))
fill.set_actor_label("RealCitySF_Daylight_Fill")
fill.set_actor_rotation(unreal.Rotator(-35, 145, 0), False)
fill_component = fill.get_component_by_class(unreal.DirectionalLightComponent)
fill_component.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
fill_component.set_editor_property("intensity", 7000.0)
fill_component.set_editor_property("light_color", unreal.Color(184, 209, 255, 255))
fill_component.set_editor_property("cast_shadows", False)

sky = actor_by_label_or_class(actors, "SkyLight")
if sky is None:
    sky = unreal.EditorLevelLibrary.spawn_actor_from_class(
        unreal.SkyLight, unreal.Vector(0, 0, 500), unreal.Rotator(0, 0, 0))
sky.set_actor_label("RealCitySF_Daylight_Sky")
sky_component = sky.get_component_by_class(unreal.SkyLightComponent)
sky_component.set_editor_property("mobility", unreal.ComponentMobility.MOVABLE)
sky_component.set_editor_property("intensity", 2.0)
sky_component.set_editor_property("lower_hemisphere_is_black", False)
sky_component.set_editor_property("real_time_capture", True)
sky_component.recapture_sky()

post = None
for actor in actors:
    if actor.get_class().get_name() == "PostProcessVolume" and actor.get_actor_label() == "RealCitySF_Daylight_PostProcess":
        post = actor
        break
if post is None:
    post = unreal.EditorLevelLibrary.spawn_actor_from_class(
        unreal.PostProcessVolume, unreal.Vector(0, 0, 0), unreal.Rotator(0, 0, 0))
post.set_actor_label("RealCitySF_Daylight_PostProcess")
post.set_editor_property("unbound", True)
post.set_editor_property("priority", 100.0)
post.set_editor_property("blend_weight", 1.0)
settings = post.get_editor_property("settings")
settings.set_editor_property("override_auto_exposure_method", True)
settings.set_editor_property("auto_exposure_method", unreal.AutoExposureMethod.AEM_MANUAL)
settings.set_editor_property("override_auto_exposure_bias", True)
settings.set_editor_property("auto_exposure_bias", 1.5)
post.set_editor_property("settings", settings)

unreal.EditorLevelLibrary.save_current_level()
unreal.log("RealCitySF daylight actors and exposure saved")
