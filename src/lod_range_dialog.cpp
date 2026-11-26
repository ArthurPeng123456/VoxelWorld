#include "lod_range_dialog.hpp"

#include <stdexcept>
#include <string>

namespace voxel {
namespace {

constexpr int radiusField = 1000;
constexpr int preview = 1001;
constexpr int summary = 1002;
constexpr int resetRanges = 1003;

struct RangeEditor { std::uint32_t radius; int densityShift; };

void addControl(HWND dialog, const wchar_t* type, const wchar_t* label, DWORD style,
                int id, int x, int y, int width, int height) {
    RECT rect{x, y, x + width, y + height};
    MapDialogRect(dialog, &rect);
    const auto control = CreateWindowExW(0, type, label, WS_CHILD | WS_VISIBLE | style,
        rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top,
        dialog, reinterpret_cast<HMENU>(INT_PTR(id)), GetModuleHandleW(nullptr), nullptr);
    SendMessageW(control, WM_SETFONT, reinterpret_cast<WPARAM>(GetStockObject(DEFAULT_GUI_FONT)), FALSE);
}

std::uint32_t readRadius(HWND dialog) {
    wchar_t text[12]{};
    GetDlgItemTextW(dialog, radiusField, text, int(std::size(text)));
    std::uint64_t value = 0;
    for (const wchar_t* p = text; *p; ++p) {
        if (*p < L'0' || *p > L'9') return 0;
        value = value * 10 + unsigned(*p - L'0');
    }
    return value < fieldSideLength ? std::uint32_t(value) : 0;
}

void refreshPreview(HWND dialog) {
    const auto radius = readRadius(dialog);
    const int count = lodWindowCount(radius);
    const HWND list = GetDlgItem(dialog, preview);
    SendMessageW(list, LB_RESETCONTENT, 0, 0);
    EnableWindow(GetDlgItem(dialog, IDOK), count != 0);
    if (!count) {
        SetDlgItemTextW(dialog, summary, L"Enter a power of two in meters smaller than the field width.");
        return;
    }
    const auto status = std::to_wstring(count + 1) + L" levels including the field-wide fallback";
    SetDlgItemTextW(dialog, summary, status.c_str());
    for (int lod = 0; lod <= count; ++lod) {
        const auto extent = std::uint64_t(radius) << lod;
        auto label = L"LOD " + std::to_wstring(lod) + L"    " + std::to_wstring(extent) + L" m";
        if (lod == count) label += L"    (field-wide fallback)";
        SendMessageW(list, LB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label.c_str()));
    }
}

INT_PTR CALLBACK rangeDialog(HWND dialog, UINT message, WPARAM w, LPARAM l) {
    if (message == WM_INITDIALOG) {
        const auto& editor = *reinterpret_cast<RangeEditor*>(l);
        SetWindowLongPtrW(dialog, DWLP_USER, l);
        SetWindowTextW(dialog, L"LOD ranges");
        addControl(dialog, L"STATIC", L"LOD0 radius (m)", 0, -1, 10, 11, 74, 10);
        addControl(dialog, L"EDIT", L"", WS_TABSTOP | WS_BORDER | ES_NUMBER | ES_AUTOHSCROLL, radiusField, 87, 8, 74, 14);
        addControl(dialog, L"STATIC", L"Power of two", 0, -1, 168, 11, 95, 10);
        SendDlgItemMessageW(dialog, radiusField, EM_SETLIMITTEXT, 10, 0);
        const auto field = std::to_wstring(fieldCellsPerAxis(editor.densityShift)) + L" cells per axis   |   " +
                           std::to_wstring(std::uint64_t(fieldSideLength)) + L" m per side";
        addControl(dialog, L"STATIC", field.c_str(), 0, -1, 10, 30, 252, 11);
        addControl(dialog, L"STATIC", L"", 0, summary, 10, 45, 252, 11);
        addControl(dialog, L"LISTBOX", L"", WS_TABSTOP | WS_BORDER | WS_VSCROLL | LBS_NOINTEGRALHEIGHT,
                   preview, 10, 62, 252, 110);
        addControl(dialog, L"STATIC", L"Apply builds terrain in the background. Resident block counts change when the new data is ready.",
                   0, -1, 10, 177, 252, 19);
        addControl(dialog, L"BUTTON", L"Defaults", WS_TABSTOP | BS_PUSHBUTTON, resetRanges, 10, 199, 55, 15);
        addControl(dialog, L"BUTTON", L"Apply", WS_TABSTOP | BS_DEFPUSHBUTTON, IDOK, 147, 199, 55, 15);
        addControl(dialog, L"BUTTON", L"Cancel", WS_TABSTOP | BS_PUSHBUTTON, IDCANCEL, 207, 199, 55, 15);
        SendMessageW(dialog, DM_SETDEFID, IDOK, 0);
        SetDlgItemInt(dialog, radiusField, editor.radius, FALSE);
        refreshPreview(dialog);
        return TRUE;
    }
    if (message != WM_COMMAND) return FALSE;
    if (LOWORD(w) == IDCANCEL) { EndDialog(dialog, IDCANCEL); return TRUE; }
    if (LOWORD(w) == resetRanges) { SetDlgItemInt(dialog, radiusField, defaultLod0Radius, FALSE); return TRUE; }
    if (LOWORD(w) == radiusField && HIWORD(w) == EN_CHANGE) { refreshPreview(dialog); return TRUE; }
    if (LOWORD(w) != IDOK) return FALSE;

    const auto radius = readRadius(dialog);
    if (!validLod0Radius(radius)) return TRUE;
    reinterpret_cast<RangeEditor*>(GetWindowLongPtrW(dialog, DWLP_USER))->radius = radius;
    EndDialog(dialog, IDOK);
    return TRUE;
}

} // namespace

bool editLodRanges(HWND owner, std::uint32_t& lod0Radius, int densityShift) {
    const struct alignas(DWORD) {
        DLGTEMPLATE dialog;
        WORD menu, windowClass, title;
    } definition{{WS_POPUP | WS_CAPTION | WS_SYSMENU | DS_MODALFRAME | DS_CENTER,
                   0, 0, 0, 0, 272, 223}, 0, 0, 0};
    RangeEditor editor{lod0Radius, densityShift};
    const auto result = DialogBoxIndirectParamW(GetModuleHandleW(nullptr), &definition.dialog, owner,
                                                rangeDialog, reinterpret_cast<LPARAM>(&editor));
    if (result == -1) throw std::runtime_error("Create LOD range dialog failed");
    if (result != IDOK) return false;
    lod0Radius = editor.radius;
    return true;
}

} // namespace voxel
