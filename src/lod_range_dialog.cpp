#include "lod_range_dialog.hpp"

#include <stdexcept>
#include <string>
#include <sstream>
#include <iomanip>

namespace voxel {
namespace {

constexpr int cellsField = 1000;
constexpr int preview = 1001;
constexpr int summary = 1002;
constexpr int resetRanges = 1003;

struct RangeEditor { std::uint32_t cells; int densityShift; };

void addControl(HWND dialog, const wchar_t* type, const wchar_t* label, DWORD style,
                int id, int x, int y, int width, int height) {
    RECT rect{x, y, x + width, y + height};
    MapDialogRect(dialog, &rect);
    const auto control = CreateWindowExW(0, type, label, WS_CHILD | WS_VISIBLE | style,
        rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top,
        dialog, reinterpret_cast<HMENU>(INT_PTR(id)), GetModuleHandleW(nullptr), nullptr);
    SendMessageW(control, WM_SETFONT, reinterpret_cast<WPARAM>(GetStockObject(DEFAULT_GUI_FONT)), FALSE);
}

std::uint32_t readCells(HWND dialog) {
    wchar_t text[12]{};
    GetDlgItemTextW(dialog, cellsField, text, int(std::size(text)));
    std::uint64_t value = 0;
    for (const wchar_t* p = text; *p; ++p) {
        if (*p < L'0' || *p > L'9') return 0;
        value = value * 10 + unsigned(*p - L'0');
    }
    return value <= UINT32_MAX && validLod0Cells(std::uint32_t(value)) ? std::uint32_t(value) : 0;
}

void refreshPreview(HWND dialog) {
    const auto cells = readCells(dialog);
    const auto& editor = *reinterpret_cast<RangeEditor*>(GetWindowLongPtrW(dialog, DWLP_USER));
    const int count = lodWindowCount(cells, editor.densityShift);
    const HWND list = GetDlgItem(dialog, preview);
    SendMessageW(list, LB_RESETCONTENT, 0, 0);
    EnableWindow(GetDlgItem(dialog, IDOK), count != 0);
    if (!count) {
        SetDlgItemTextW(dialog, summary, L"Enter a power-of-two cell count per edge (32 to 4194304).");
        return;
    }
    std::wostringstream status;
    status << std::setprecision(12) << L"LOD0: " << cells << L" cells/edge, " << lodSideLength(cells, editor.densityShift)
           << L" m side, " << lodSideLength(cells, editor.densityShift) / 2 << L" m half-extent";
    SetDlgItemTextW(dialog, summary, status.str().c_str());
    for (int lod = 0; lod <= count; ++lod) {
        std::wostringstream label;
        label << std::setprecision(12) << L"LOD " << lod << L"   step " << std::ldexp(gridSpacing(editor.densityShift), lod)
              << L" m   side " << lodSideLength(cells, editor.densityShift, lod) << L" m";
        if (lod == count) label << L"   (field-wide fallback)";
        SendMessageW(list, LB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label.str().c_str()));
    }
}

INT_PTR CALLBACK rangeDialog(HWND dialog, UINT message, WPARAM w, LPARAM l) {
    if (message == WM_INITDIALOG) {
        const auto& editor = *reinterpret_cast<RangeEditor*>(l);
        SetWindowLongPtrW(dialog, DWLP_USER, l);
        SetWindowTextW(dialog, L"LOD ranges");
        addControl(dialog, L"STATIC", L"LOD0 cells / edge", 0, -1, 10, 11, 85, 10);
        addControl(dialog, L"EDIT", L"", WS_TABSTOP | WS_BORDER | ES_NUMBER | ES_AUTOHSCROLL, cellsField, 98, 8, 74, 14);
        addControl(dialog, L"STATIC", L"Power of two, >=32", 0, -1, 180, 11, 115, 10);
        SendDlgItemMessageW(dialog, cellsField, EM_SETLIMITTEXT, 10, 0);
        const auto field = std::to_wstring(fieldCellsPerAxis(editor.densityShift)) + L" cells per axis   |   " +
                           std::to_wstring(std::uint64_t(fieldSideLength)) + L" m per side";
        addControl(dialog, L"STATIC", field.c_str(), 0, -1, 10, 30, 320, 11);
        addControl(dialog, L"STATIC", L"", 0, summary, 10, 45, 320, 11);
        addControl(dialog, L"LISTBOX", L"", WS_TABSTOP | WS_BORDER | WS_VSCROLL | LBS_NOINTEGRALHEIGHT,
                   preview, 10, 62, 320, 110);
        addControl(dialog, L"STATIC", L"Cells are intervals (N cells = N+1 sample points). Each LOD doubles spacing and side length. Rolling windows include block alignment and hysteresis padding.",
                   0, -1, 10, 177, 320, 28);
        addControl(dialog, L"STATIC", L"4096 cells/edge is expensive: even one flat surface has 16.8 million cells. Apply builds in the background; counts update after publication.",
                   0, -1, 10, 208, 320, 28);
        addControl(dialog, L"BUTTON", L"Defaults", WS_TABSTOP | BS_PUSHBUTTON, resetRanges, 10, 242, 55, 15);
        addControl(dialog, L"BUTTON", L"Apply", WS_TABSTOP | BS_DEFPUSHBUTTON, IDOK, 215, 242, 55, 15);
        addControl(dialog, L"BUTTON", L"Cancel", WS_TABSTOP | BS_PUSHBUTTON, IDCANCEL, 275, 242, 55, 15);
        SendMessageW(dialog, DM_SETDEFID, IDOK, 0);
        SetDlgItemInt(dialog, cellsField, editor.cells, FALSE);
        refreshPreview(dialog);
        return TRUE;
    }
    if (message != WM_COMMAND) return FALSE;
    if (LOWORD(w) == IDCANCEL) { EndDialog(dialog, IDCANCEL); return TRUE; }
    if (LOWORD(w) == resetRanges) { SetDlgItemInt(dialog, cellsField, defaultLod0Cells, FALSE); return TRUE; }
    if (LOWORD(w) == cellsField && HIWORD(w) == EN_CHANGE) { refreshPreview(dialog); return TRUE; }
    if (LOWORD(w) != IDOK) return FALSE;

    const auto cells = readCells(dialog);
    if (!validLod0Cells(cells)) return TRUE;
    reinterpret_cast<RangeEditor*>(GetWindowLongPtrW(dialog, DWLP_USER))->cells = cells;
    EndDialog(dialog, IDOK);
    return TRUE;
}

} // namespace

bool editLodRanges(HWND owner, std::uint32_t& lod0Cells, int densityShift) {
    const struct alignas(DWORD) {
        DLGTEMPLATE dialog;
        WORD menu, windowClass, title;
    } definition{{WS_POPUP | WS_CAPTION | WS_SYSMENU | DS_MODALFRAME | DS_CENTER,
                   0, 0, 0, 0, 340, 267}, 0, 0, 0};
    RangeEditor editor{lod0Cells, densityShift};
    const auto result = DialogBoxIndirectParamW(GetModuleHandleW(nullptr), &definition.dialog, owner,
                                                rangeDialog, reinterpret_cast<LPARAM>(&editor));
    if (result == -1) throw std::runtime_error("Create LOD range dialog failed");
    if (result != IDOK) return false;
    lod0Cells = editor.cells;
    return true;
}

} // namespace voxel
