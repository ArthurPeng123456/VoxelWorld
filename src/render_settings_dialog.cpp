#include "render_settings_dialog.hpp"

#include <commctrl.h>
#include <cwchar>
#include <vector>

namespace voxel {
namespace {

enum Page { ShadowPage, AtmospherePage, CloudPage, TaaPage, HdrPage, PageCount };
enum Field {
    ShadowEnabled = 1200, Resolution, ShadowMemory, AtmosphereEnabled, ToneMapper, Status, Defaults,
    Tabs = 1210, ShadowFirst = 1220, AtmosphereFirst = 1230, HdrFirst = 1240,
    CloudEnabled = 1245, CloudQuality, CloudFirst = 1250,
    CloudTemporalEnabled = 1265, CloudPattern, CloudTemporalFirst = 1270,
    TaaEnabled = 1280, TaaSamples, TaaFirst = 1290
};
struct DialogState {
    RenderSettings* settings;
    RenderSettings defaults;
    std::array<std::vector<HWND>, PageCount> pages;
    int buildingPage{-1};
};
DialogState& state(HWND dialog) { return *reinterpret_cast<DialogState*>(GetWindowLongPtrW(dialog, DWLP_USER)); }

void addControl(HWND dialog, const wchar_t* type, const wchar_t* text, DWORD style,
                int id, int x, int y, int width, int height) {
    RECT rect{x, y, x + width, y + height};
    MapDialogRect(dialog, &rect);
    const auto control = CreateWindowExW(0, type, text, WS_CHILD | WS_VISIBLE | style,
        rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top,
        dialog, reinterpret_cast<HMENU>(INT_PTR(id)), GetModuleHandleW(nullptr), nullptr);
    SendMessageW(control, WM_SETFONT, reinterpret_cast<WPARAM>(GetStockObject(DEFAULT_GUI_FONT)), FALSE);
    if (state(dialog).buildingPage >= 0) state(dialog).pages[std::size_t(state(dialog).buildingPage)].push_back(control);
}
void selectPage(HWND dialog, int page) {
    if (page < 0 || page >= PageCount) return;
    for (int i = 0; i < PageCount; ++i) for (HWND control : state(dialog).pages[std::size_t(i)])
        ShowWindow(control, i == page ? SW_SHOW : SW_HIDE);
    TabCtrl_SetCurSel(GetDlgItem(dialog, Tabs), page);
}

template<std::size_t N>
void addFields(HWND dialog, const std::array<RenderParameter, N>& parameters, int first, int x, int y) {
    for (std::size_t i = 0; i < N; ++i) {
        const int row = y + int(i) * 18, id = first + int(i);
        addControl(dialog, L"STATIC", parameters[i].label, 0, -1, x, row + 2, 119, 11);
        addControl(dialog, L"EDIT", L"", WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL, id, x + 121, row, 58, 13);
        SendDlgItemMessageW(dialog, id, EM_SETLIMITTEXT, 24, 0);
    }
}
template<std::size_t N>
void writeFields(HWND dialog, const std::array<RenderParameter, N>& parameters, int first, const RenderSettings& settings) {
    for (std::size_t i = 0; i < N; ++i) {
        wchar_t text[32]{};
        std::swprintf(text, std::size(text), L"%.9g", settings.*parameters[i].value);
        SetDlgItemTextW(dialog, first + int(i), text);
    }
}
template<std::size_t N>
bool readFields(HWND dialog, const std::array<RenderParameter, N>& parameters, int first, int page, RenderSettings& settings) {
    for (std::size_t i = 0; i < N; ++i) {
        const int id = first + int(i);
        wchar_t text[32]{};
        GetDlgItemTextW(dialog, id, text, int(std::size(text)));
        const auto& field = parameters[i];
        const auto value = readRenderParameter(text, field);
        if (!value) {
            wchar_t hint[160]{};
            std::swprintf(hint, std::size(hint), L"%ls accepts values from %g to %g.", field.label, field.minimum, field.maximum);
            SetDlgItemTextW(dialog, Status, hint);
            selectPage(dialog, page);
            SetFocus(GetDlgItem(dialog, id));
            SendDlgItemMessageW(dialog, id, EM_SETSEL, 0, -1);
            return false;
        }
        settings.*field.value = *value;
    }
    return true;
}
void dependencies(HWND dialog) {
    const bool taa = IsDlgButtonChecked(dialog, TaaEnabled) == BST_CHECKED;
    const bool cloud = taa && IsDlgButtonChecked(dialog, CloudEnabled) == BST_CHECKED;
    const bool temporal = cloud && IsDlgButtonChecked(dialog, CloudTemporalEnabled) == BST_CHECKED;
    EnableWindow(GetDlgItem(dialog, CloudTemporalEnabled), cloud);
    EnableWindow(GetDlgItem(dialog, CloudPattern), temporal);
    for (int i = 0; i < int(cloudTemporalParameters.size()); ++i) EnableWindow(GetDlgItem(dialog, CloudTemporalFirst + i), temporal);
    EnableWindow(GetDlgItem(dialog, TaaSamples), taa);
    for (int i = 0; i < int(taaParameters.size()); ++i) EnableWindow(GetDlgItem(dialog, TaaFirst + i), taa);
}
void updateMemory(HWND dialog) {
    const auto row = SendDlgItemMessageW(dialog, Resolution, CB_GETCURSEL, 0, 0);
    if (row < 0 || row >= LRESULT(shadowResolutions.size())) return;
    const int edge = shadowResolutions[std::size_t(row)];
    const auto text = L"Four depth maps   ~" + std::to_wstring(edge * edge * 16 / (1024 * 1024)) + L" MiB";
    SetDlgItemTextW(dialog, ShadowMemory, text.c_str());
}
void writeSettings(HWND dialog, const RenderSettings& settings) {
    CheckDlgButton(dialog, ShadowEnabled, settings.shadowsEnabled ? BST_CHECKED : BST_UNCHECKED);
    CheckDlgButton(dialog, AtmosphereEnabled, settings.atmosphereEnabled ? BST_CHECKED : BST_UNCHECKED);
    CheckDlgButton(dialog, CloudEnabled, settings.cloudsEnabled ? BST_CHECKED : BST_UNCHECKED);
    CheckDlgButton(dialog, TaaEnabled, settings.taaEnabled ? BST_CHECKED : BST_UNCHECKED);
    CheckDlgButton(dialog, CloudTemporalEnabled, settings.cloudTemporalEnabled ? BST_CHECKED : BST_UNCHECKED);
    for (std::size_t i = 0; i < cloudStepCounts.size(); ++i) if (settings.cloudSteps == cloudStepCounts[i])
        SendDlgItemMessageW(dialog, CloudQuality, CB_SETCURSEL, WPARAM(i), 0);
    for (std::size_t i = 0; i < shadowResolutions.size(); ++i) if (settings.shadowResolution == shadowResolutions[i])
        SendDlgItemMessageW(dialog, Resolution, CB_SETCURSEL, WPARAM(i), 0);
    for (std::size_t i = 0; i < taaSampleCounts.size(); ++i) if (settings.taaSamples == taaSampleCounts[i])
        SendDlgItemMessageW(dialog, TaaSamples, CB_SETCURSEL, WPARAM(i), 0);
    SendDlgItemMessageW(dialog, CloudPattern, CB_SETCURSEL, WPARAM(settings.cloudSampling), 0);
    SendDlgItemMessageW(dialog, ToneMapper, CB_SETCURSEL, WPARAM(settings.toneMapping), 0);
    writeFields(dialog, shadowParameters, ShadowFirst, settings);
    writeFields(dialog, atmosphereParameters, AtmosphereFirst, settings);
    writeFields(dialog, cloudParameters, CloudFirst, settings);
    writeFields(dialog, hdrParameters, HdrFirst, settings);
    writeFields(dialog, taaParameters, TaaFirst, settings);
    writeFields(dialog, cloudTemporalParameters, CloudTemporalFirst, settings);
    SetDlgItemTextW(dialog, Status, L"");
    updateMemory(dialog); dependencies(dialog);
}

INT_PTR CALLBACK renderDialog(HWND dialog, UINT message, WPARAM w, LPARAM l) {
    if (message == WM_INITDIALOG) {
        SetWindowLongPtrW(dialog, DWLP_USER, l);
        SetWindowTextW(dialog, L"Render settings");
        addControl(dialog, WC_TABCONTROLW, L"", WS_TABSTOP, Tabs, 8, 7, 428, 242);
        int index = 0;
        for (const auto* label : {L"CSM", L"Atmosphere", L"Clouds", L"TAA", L"HDR"}) {
            TCITEMW item{}; item.mask = TCIF_TEXT; item.pszText = const_cast<wchar_t*>(label);
            TabCtrl_InsertItem(GetDlgItem(dialog, Tabs), index++, &item);
        }
        const auto dropdown = WS_TABSTOP | WS_VSCROLL | CBS_DROPDOWNLIST;
        state(dialog).buildingPage = ShadowPage;
        addControl(dialog, L"BUTTON", L"Enable CSM shadows", WS_TABSTOP | BS_AUTOCHECKBOX, ShadowEnabled, 18, 40, 180, 12);
        addControl(dialog, L"STATIC", L"Shadow map per cascade", 0, -1, 18, 61, 119, 11);
        addControl(dialog, L"COMBOBOX", L"", dropdown, Resolution, 139, 58, 58, 65);
        for (int edge : shadowResolutions) {
            const auto label = std::to_wstring(edge) + L" x " + std::to_wstring(edge);
            SendDlgItemMessageW(dialog, Resolution, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label.c_str()));
        }
        addFields(dialog, shadowParameters, ShadowFirst, 18, 82);
        addControl(dialog, L"STATIC", L"", 0, ShadowMemory, 18, 184, 179, 11);
        addControl(dialog, L"STATIC", L"Near cascade range reserves shadow detail close to the camera. Smaller values sharpen nearby shadows.\n\nThe near range limits the split distribution; the distance cap still controls distant coverage.\n\nHigher map resolution uses more GPU memory.", 0, -1, 220, 45, 190, 130);

        state(dialog).buildingPage = AtmospherePage;
        addControl(dialog, L"BUTTON", L"Enable atmosphere [H]", WS_TABSTOP | BS_AUTOCHECKBOX, AtmosphereEnabled, 18, 40, 180, 12);
        addFields(dialog, atmosphereParameters, AtmosphereFirst, 18, 64);
        addControl(dialog, L"STATIC", L"Thickness stretches the outer boundary and density profile together.\n\nScale heights are specified for the 100 km reference thickness.", 0, -1, 220, 45, 190, 90);

        state(dialog).buildingPage = CloudPage;
        addControl(dialog, L"BUTTON", L"Enable clouds [V]", WS_TABSTOP | BS_AUTOCHECKBOX, CloudEnabled, 18, 40, 180, 12);
        addFields(dialog, cloudParameters, CloudFirst, 18, 64);
        addControl(dialog, L"STATIC", L"Raymarch quality", 0, -1, 220, 40, 190, 11);
        addControl(dialog, L"COMBOBOX", L"", dropdown, CloudQuality, 220, 55, 190, 65);
        for (const auto* label : {L"Low (32 samples)", L"Medium (64 samples)", L"High (96 samples)"})
            SendDlgItemMessageW(dialog, CloudQuality, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label));
        addControl(dialog, L"BUTTON", L"Temporal cloud reconstruction", WS_TABSTOP | BS_AUTOCHECKBOX, CloudTemporalEnabled, 220, 82, 190, 12);
        addControl(dialog, L"STATIC", L"Pixels raymarched per frame", 0, -1, 220, 102, 190, 11);
        addControl(dialog, L"COMBOBOX", L"", dropdown, CloudPattern, 220, 116, 190, 65);
        for (const auto* label : {L"Full sampling", L"2-phase checkerboard (1/2 pixels)", L"4-phase 2x2 (1/4 pixels)"})
            SendDlgItemMessageW(dialog, CloudPattern, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label));
        addFields(dialog, cloudTemporalParameters, CloudTemporalFirst, 220, 141);
        addControl(dialog, L"STATIC", L"Sparse sampling requires TAA. Invalid history is raymarched immediately. Turning TAA off restores full sampling.", 0, -1, 220, 203, 190, 34);

        state(dialog).buildingPage = TaaPage;
        addControl(dialog, L"BUTTON", L"Enable temporal anti-aliasing", WS_TABSTOP | BS_AUTOCHECKBOX, TaaEnabled, 18, 40, 190, 12);
        addFields(dialog, taaParameters, TaaFirst, 18, 64);
        addControl(dialog, L"STATIC", L"Jitter cycle", 0, -1, 220, 40, 190, 11);
        addControl(dialog, L"COMBOBOX", L"", dropdown, TaaSamples, 220, 55, 190, 65);
        for (int samples : taaSampleCounts) {
            const auto label = std::to_wstring(samples) + L" frames";
            SendDlgItemMessageW(dialog, TaaSamples, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label.c_str()));
        }
        addControl(dialog, L"STATIC", L"More history reduces flicker but may increase trails.\n\nLarger variance gamma preserves more history.\n\nMotion response reduces history during movement.\n\nSharpening affects the final output only. Apply also clears accumulated history.", 0, -1, 220, 88, 190, 130);

        state(dialog).buildingPage = HdrPage;
        addFields(dialog, hdrParameters, HdrFirst, 18, 46);
        addControl(dialog, L"STATIC", L"Tone mapping", 0, -1, 18, 75, 119, 11);
        addControl(dialog, L"COMBOBOX", L"", dropdown, ToneMapper, 139, 72, 179, 63);
        for (const auto* label : {L"ACES (filmic)", L"Reinhard", L"Linear (clamp)"})
            SendDlgItemMessageW(dialog, ToneMapper, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label));
        addControl(dialog, L"STATIC", L"Temporal filtering operates on linear HDR before exposure and tone mapping. Display output remains sRGB/SDR.", 0, -1, 18, 111, 380, 45);

        state(dialog).buildingPage = -1;
        addControl(dialog, L"STATIC", L"", 0, Status, 10, 255, 422, 11);
        addControl(dialog, L"BUTTON", L"Defaults", WS_TABSTOP | BS_PUSHBUTTON, Defaults, 10, 273, 58, 15);
        addControl(dialog, L"BUTTON", L"Apply", WS_TABSTOP | BS_DEFPUSHBUTTON, IDOK, 308, 273, 58, 15);
        addControl(dialog, L"BUTTON", L"Cancel", WS_TABSTOP | BS_PUSHBUTTON, IDCANCEL, 374, 273, 58, 15);
        SendMessageW(dialog, DM_SETDEFID, IDOK, 0);
        writeSettings(dialog, *state(dialog).settings);
        selectPage(dialog, ShadowPage);
        return TRUE;
    }
    if (message == WM_NOTIFY && reinterpret_cast<NMHDR*>(l)->idFrom == Tabs && reinterpret_cast<NMHDR*>(l)->code == TCN_SELCHANGE) {
        selectPage(dialog, TabCtrl_GetCurSel(GetDlgItem(dialog, Tabs))); return TRUE;
    }
    if (message != WM_COMMAND) return FALSE;
    const auto id = LOWORD(w);
    if (id == IDCANCEL) { EndDialog(dialog, IDCANCEL); return TRUE; }
    if (id == Defaults) { writeSettings(dialog, state(dialog).defaults); return TRUE; }
    if (id == Resolution && HIWORD(w) == CBN_SELCHANGE) { updateMemory(dialog); return TRUE; }
    if ((id == TaaEnabled || id == CloudEnabled || id == CloudTemporalEnabled) && HIWORD(w) == BN_CLICKED) { dependencies(dialog); return TRUE; }
    if (id != IDOK) return FALSE;
    RenderSettings next = *state(dialog).settings;
    next.shadowsEnabled = IsDlgButtonChecked(dialog, ShadowEnabled) == BST_CHECKED;
    next.atmosphereEnabled = IsDlgButtonChecked(dialog, AtmosphereEnabled) == BST_CHECKED;
    next.cloudsEnabled = IsDlgButtonChecked(dialog, CloudEnabled) == BST_CHECKED;
    next.taaEnabled = IsDlgButtonChecked(dialog, TaaEnabled) == BST_CHECKED;
    next.cloudTemporalEnabled = IsDlgButtonChecked(dialog, CloudTemporalEnabled) == BST_CHECKED;
    const auto quality = SendDlgItemMessageW(dialog, CloudQuality, CB_GETCURSEL, 0, 0);
    const auto resolution = SendDlgItemMessageW(dialog, Resolution, CB_GETCURSEL, 0, 0);
    const auto samples = SendDlgItemMessageW(dialog, TaaSamples, CB_GETCURSEL, 0, 0);
    const auto pattern = SendDlgItemMessageW(dialog, CloudPattern, CB_GETCURSEL, 0, 0);
    const auto tone = SendDlgItemMessageW(dialog, ToneMapper, CB_GETCURSEL, 0, 0);
    if (quality < 0 || quality >= LRESULT(cloudStepCounts.size()) || resolution < 0 || resolution >= LRESULT(shadowResolutions.size()) ||
        samples < 0 || samples >= LRESULT(taaSampleCounts.size()) || pattern < 0 || pattern > 2 || tone < 0 || tone > int(ToneMapping::Linear)) return TRUE;
    next.cloudSteps = cloudStepCounts[std::size_t(quality)]; next.shadowResolution = shadowResolutions[std::size_t(resolution)];
    next.taaSamples = taaSampleCounts[std::size_t(samples)]; next.cloudSampling = int(pattern); next.toneMapping = ToneMapping(tone);
    if (!readFields(dialog, shadowParameters, ShadowFirst, ShadowPage, next) ||
        !readFields(dialog, atmosphereParameters, AtmosphereFirst, AtmospherePage, next) ||
        !readFields(dialog, cloudParameters, CloudFirst, CloudPage, next) ||
        !readFields(dialog, cloudTemporalParameters, CloudTemporalFirst, CloudPage, next) ||
        !readFields(dialog, taaParameters, TaaFirst, TaaPage, next) || !readFields(dialog, hdrParameters, HdrFirst, HdrPage, next)) return TRUE;
    *state(dialog).settings = next;
    EndDialog(dialog, IDOK); return TRUE;
}

} // namespace

bool editRenderSettings(HWND owner, RenderSettings& settings, const RenderSettings& defaults) {
    INITCOMMONCONTROLSEX controls{sizeof(controls), ICC_TAB_CLASSES};
    if (!InitCommonControlsEx(&controls)) throw std::runtime_error("Initialize render settings tabs failed");
    const struct alignas(DWORD) {
        DLGTEMPLATE dialog;
        WORD menu, windowClass, title;
    } definition{{WS_POPUP | WS_CAPTION | WS_SYSMENU | DS_MODALFRAME | DS_CENTER,
                   0, 0, 0, 0, 444, 300}, 0, 0, 0};
    DialogState data{&settings, defaults, {}, -1};
    const auto result = DialogBoxIndirectParamW(GetModuleHandleW(nullptr), &definition.dialog, owner,
                                                renderDialog, reinterpret_cast<LPARAM>(&data));
    if (result == -1) throw std::runtime_error("Create render settings dialog failed");
    return result == IDOK;
}

} // namespace voxel
