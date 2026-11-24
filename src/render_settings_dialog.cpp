#include "render_settings_dialog.hpp"

#include <cwchar>

namespace voxel {
namespace {

enum Field {
    ShadowEnabled = 1200, Resolution, ShadowMemory, AtmosphereEnabled, ToneMapper, Status, Defaults,
    ShadowFirst = 1220, AtmosphereFirst = 1230, HdrFirst = 1240
};

void addControl(HWND dialog, const wchar_t* type, const wchar_t* text, DWORD style,
                int id, int x, int y, int width, int height) {
    RECT rect{x, y, x + width, y + height};
    MapDialogRect(dialog, &rect);
    const auto control = CreateWindowExW(0, type, text, WS_CHILD | WS_VISIBLE | style,
        rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top,
        dialog, reinterpret_cast<HMENU>(INT_PTR(id)), GetModuleHandleW(nullptr), nullptr);
    SendMessageW(control, WM_SETFONT, reinterpret_cast<WPARAM>(GetStockObject(DEFAULT_GUI_FONT)), FALSE);
}

template<std::size_t N>
void addFields(HWND dialog, const std::array<RenderParameter, N>& parameters, int first, int x, int y) {
    for (std::size_t i = 0; i < N; ++i) {
        const int row = y + int(i) * 15, id = first + int(i);
        addControl(dialog, L"STATIC", parameters[i].label, 0, -1, x, row + 2, 111, 11);
        addControl(dialog, L"EDIT", L"", WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL, id, x + 113, row, 52, 13);
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
bool readFields(HWND dialog, const std::array<RenderParameter, N>& parameters, int first, RenderSettings& settings) {
    for (std::size_t i = 0; i < N; ++i) {
        const int id = first + int(i);
        wchar_t text[32]{};
        GetDlgItemTextW(dialog, id, text, int(std::size(text)));
        const auto& field = parameters[i];
        const auto value = readRenderParameter(text, field);
        if (!value) {
            wchar_t hint[128]{};
            std::swprintf(hint, std::size(hint), L"%ls accepts values from %g to %g.", field.label, field.minimum, field.maximum);
            SetDlgItemTextW(dialog, Status, hint);
            SetFocus(GetDlgItem(dialog, id));
            SendDlgItemMessageW(dialog, id, EM_SETSEL, 0, -1);
            return false;
        }
        settings.*field.value = *value;
    }
    return true;
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
    for (std::size_t i = 0; i < shadowResolutions.size(); ++i) if (shadowResolutions[i] == settings.shadowResolution)
        SendDlgItemMessageW(dialog, Resolution, CB_SETCURSEL, WPARAM(i), 0);
    SendDlgItemMessageW(dialog, ToneMapper, CB_SETCURSEL, WPARAM(settings.toneMapping), 0);
    writeFields(dialog, shadowParameters, ShadowFirst, settings);
    writeFields(dialog, atmosphereParameters, AtmosphereFirst, settings);
    writeFields(dialog, hdrParameters, HdrFirst, settings);
    SetDlgItemTextW(dialog, Status, L"");
    updateMemory(dialog);
}

INT_PTR CALLBACK renderDialog(HWND dialog, UINT message, WPARAM w, LPARAM l) {
    if (message == WM_INITDIALOG) {
        SetWindowLongPtrW(dialog, DWLP_USER, l);
        SetWindowTextW(dialog, L"Render settings");
        const auto dropdown = WS_TABSTOP | WS_VSCROLL | CBS_DROPDOWNLIST;
        addControl(dialog, L"BUTTON", L"CSM shadows", BS_GROUPBOX, -1, 8, 5, 186, 131);
        addControl(dialog, L"BUTTON", L"Enable shadows", WS_TABSTOP | BS_AUTOCHECKBOX, ShadowEnabled, 18, 18, 163, 12);
        addControl(dialog, L"STATIC", L"Shadow map per cascade", 0, -1, 18, 35, 105, 11);
        addControl(dialog, L"COMBOBOX", L"", dropdown, Resolution, 122, 32, 62, 65);
        for (int edge : shadowResolutions) {
            const auto label = std::to_wstring(edge) + L" x " + std::to_wstring(edge);
            SendDlgItemMessageW(dialog, Resolution, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label.c_str()));
        }
        addFields(dialog, shadowParameters, ShadowFirst, 18, 50);
        addControl(dialog, L"STATIC", L"", 0, ShadowMemory, 18, 116, 166, 11);

        addControl(dialog, L"BUTTON", L"Atmosphere", BS_GROUPBOX, -1, 200, 5, 186, 131);
        addControl(dialog, L"BUTTON", L"Enable atmosphere", WS_TABSTOP | BS_AUTOCHECKBOX, AtmosphereEnabled, 210, 18, 163, 12);
        addFields(dialog, atmosphereParameters, AtmosphereFirst, 210, 34);

        addControl(dialog, L"BUTTON", L"HDR", BS_GROUPBOX, -1, 8, 141, 378, 34);
        addFields(dialog, hdrParameters, HdrFirst, 18, 154);
        addControl(dialog, L"STATIC", L"Tone mapping", 0, -1, 210, 156, 65, 11);
        addControl(dialog, L"COMBOBOX", L"", dropdown, ToneMapper, 279, 153, 97, 63);
        for (const auto* label : {L"ACES (filmic)", L"Reinhard", L"Linear (clamp)"})
            SendDlgItemMessageW(dialog, ToneMapper, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label));

        addControl(dialog, L"STATIC", L"", 0, Status, 10, 181, 375, 11);
        addControl(dialog, L"BUTTON", L"Defaults", WS_TABSTOP | BS_PUSHBUTTON, Defaults, 10, 197, 58, 15);
        addControl(dialog, L"BUTTON", L"Apply", WS_TABSTOP | BS_DEFPUSHBUTTON, IDOK, 264, 197, 58, 15);
        addControl(dialog, L"BUTTON", L"Cancel", WS_TABSTOP | BS_PUSHBUTTON, IDCANCEL, 328, 197, 58, 15);
        SendMessageW(dialog, DM_SETDEFID, IDOK, 0);
        writeSettings(dialog, *reinterpret_cast<RenderSettings*>(l));
        return TRUE;
    }
    if (message != WM_COMMAND) return FALSE;
    if (LOWORD(w) == IDCANCEL) { EndDialog(dialog, IDCANCEL); return TRUE; }
    if (LOWORD(w) == Defaults) { writeSettings(dialog, {}); return TRUE; }
    if (LOWORD(w) == Resolution && HIWORD(w) == CBN_SELCHANGE) { updateMemory(dialog); return TRUE; }
    if (LOWORD(w) != IDOK) return FALSE;

    RenderSettings next;
    next.shadowsEnabled = IsDlgButtonChecked(dialog, ShadowEnabled) == BST_CHECKED;
    next.atmosphereEnabled = IsDlgButtonChecked(dialog, AtmosphereEnabled) == BST_CHECKED;
    const auto resolution = SendDlgItemMessageW(dialog, Resolution, CB_GETCURSEL, 0, 0);
    const auto tone = SendDlgItemMessageW(dialog, ToneMapper, CB_GETCURSEL, 0, 0);
    if (resolution < 0 || resolution >= LRESULT(shadowResolutions.size()) || tone < 0 || tone > int(ToneMapping::Linear))
        return TRUE;
    next.shadowResolution = shadowResolutions[std::size_t(resolution)];
    next.toneMapping = ToneMapping(tone);
    if (!readFields(dialog, shadowParameters, ShadowFirst, next) ||
        !readFields(dialog, atmosphereParameters, AtmosphereFirst, next) || !readFields(dialog, hdrParameters, HdrFirst, next))
        return TRUE;
    *reinterpret_cast<RenderSettings*>(GetWindowLongPtrW(dialog, DWLP_USER)) = next;
    EndDialog(dialog, IDOK);
    return TRUE;
}

} // namespace

bool editRenderSettings(HWND owner, RenderSettings& settings) {
    const struct alignas(DWORD) {
        DLGTEMPLATE dialog;
        WORD menu, windowClass, title;
    } definition{{WS_POPUP | WS_CAPTION | WS_SYSMENU | DS_MODALFRAME | DS_CENTER,
                   0, 0, 0, 0, 394, 223}, 0, 0, 0};
    const auto result = DialogBoxIndirectParamW(GetModuleHandleW(nullptr), &definition.dialog, owner,
                                                renderDialog, reinterpret_cast<LPARAM>(&settings));
    if (result == -1) throw std::runtime_error("Create render settings dialog failed");
    return result == IDOK;
}

} // namespace voxel
