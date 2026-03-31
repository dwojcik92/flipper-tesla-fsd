#include "../tesla_fsd_app.h"
#include "../scenes_config/app_scene_functions.h"

void tesla_fsd_scene_about_on_enter(void* context) {
    TeslaFSDApp* app = context;

    widget_reset(app->widget);
    widget_add_string_element(
        app->widget, 64, 2, AlignCenter, AlignTop, FontPrimary,
        "Tesla FSD Unlock");

    widget_add_string_element(
        app->widget, 64, 16, AlignCenter, AlignTop, FontSecondary,
        "v" TESLA_FSD_VERSION);

    widget_add_string_multiline_element(
        app->widget, 64, 30, AlignCenter, AlignTop, FontSecondary,
        "Unlocks FSD on HW3/HW4\n"
        "via CAN bus injection.\n"
        "Requires CAN Add-On.\n"
        "github.com/hypery11/flipper-tesla-fsd");

    view_dispatcher_switch_to_view(app->view_dispatcher, TeslaFSDViewWidget);
}

bool tesla_fsd_scene_about_on_event(void* context, SceneManagerEvent event) {
    UNUSED(context);
    UNUSED(event);
    return false;
}

void tesla_fsd_scene_about_on_exit(void* context) {
    TeslaFSDApp* app = context;
    widget_reset(app->widget);
}
