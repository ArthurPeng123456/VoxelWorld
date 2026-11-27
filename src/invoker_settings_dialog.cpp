#include "invoker_settings_dialog.hpp"

#include <iomanip>
#include <sstream>

namespace voxel {
namespace {

enum Field { Count = 1000, Mode, Radius, Cruise, Return, Control, Validation, Defaults };
struct Editor { InvokerSettings settings; };

HWND add(HWND dialog, const wchar_t* type, const wchar_t* label, DWORD style,
         int id, int x, int y, int width, int height) {
    RECT rect{x, y, x + width, y + height};
    MapDialogRect(dialog, &rect);
    const auto control = CreateWindowExW(0, type, label, WS_CHILD | WS_VISIBLE | style,
        rect.left, rect.top, rect.right - rect.left, rect.bottom - rect.top,
        dialog, reinterpret_cast<HMENU>(INT_PTR(id)), GetModuleHandleW(nullptr), nullptr);
    SendMessageW(control, WM_SETFONT, SendMessageW(dialog, WM_GETFONT, 0, 0), FALSE);
    return control;
}

double number(HWND dialog, int id) {
    wchar_t text[64]{};
    GetDlgItemTextW(dialog, id, text, int(std::size(text)));
    return parseInvokerNumber(std::wstring(text));
}

bool read(HWND dialog, InvokerSettings& settings) {
    settings = reinterpret_cast<Editor*>(GetWindowLongPtrW(dialog, DWLP_USER))->settings;
    settings.mode = InvokerCruiseMode(SendDlgItemMessageW(dialog, Mode, CB_GETCURSEL, 0, 0));
    try {
        wchar_t countText[32]{};
        GetDlgItemTextW(dialog, Count, countText, int(std::size(countText)));
        settings.count = parseInvokerCount(std::wstring(countText));
        settings.cruiseSpeed = number(dialog, Cruise);
        settings.returnSpeed = number(dialog, Return);
        settings.controlSpeed = number(dialog, Control);
        if (settings.mode == InvokerCruiseMode::Area) settings.patrolRadius = number(dialog, Radius);
        else {
            try {
                const double radius = number(dialog, Radius);
                if (radius >= 1 && radius <= 1000000) settings.patrolRadius = radius;
            } catch (const std::exception&) {}
        }
    } catch (const std::exception&) { return false; }
    return validInvokerSettings(settings);
}

void refresh(HWND dialog) {
    const bool area = SendDlgItemMessageW(dialog, Mode, CB_GETCURSEL, 0, 0) == 0;
    EnableWindow(GetDlgItem(dialog, Radius), area);
    InvokerSettings draft;
    const bool valid = read(dialog, draft);
    EnableWindow(GetDlgItem(dialog, IDOK), valid);
    SetDlgItemTextW(dialog, Validation, valid ? L"" :
        L"Count: integer 1-10000; radius: 1-1000000 m; cruise/return: 0-1000 m/s; controlled: 0.1-1000 m/s.");
}

void populate(HWND dialog, const InvokerSettings& settings) {
    SetDlgItemInt(dialog, Count, settings.count, FALSE);
    SendDlgItemMessageW(dialog, Mode, CB_SETCURSEL, int(settings.mode), 0);
    for (auto field : {Radius, Cruise, Return, Control}) {
        const double value = field == Radius ? settings.patrolRadius : field == Cruise ? settings.cruiseSpeed
            : field == Return ? settings.returnSpeed : settings.controlSpeed;
        std::wostringstream text;
        text << std::setprecision(12) << value;
        SetDlgItemTextW(dialog, field, text.str().c_str());
    }
    refresh(dialog);
}

INT_PTR CALLBACK procedure(HWND dialog, UINT message, WPARAM w, LPARAM l) {
    if (message == WM_INITDIALOG) {
        SetWindowLongPtrW(dialog, DWLP_USER, l);
        SetWindowTextW(dialog, L"Invoker settings");
        add(dialog, L"STATIC", L"Applies to all Invokers. Existing Invokers and rigid bodies keep their positions.",
            0, -1, 10, 10, 320, 22);
        add(dialog, L"STATIC", L"Invoker count (1-10000)", 0, -1, 10, 40, 150, 12);
        const auto count = add(dialog, L"EDIT", L"", WS_TABSTOP | WS_BORDER | ES_NUMBER | ES_AUTOHSCROLL,
            Count, 175, 36, 155, 16);
        SendMessageW(count, EM_SETLIMITTEXT, 10, 0);
        add(dialog, L"STATIC", L"Autonomous cruise mode", 0, -1, 10, 66, 155, 12);
        add(dialog, L"COMBOBOX", L"", WS_TABSTOP | CBS_DROPDOWNLIST, Mode, 175, 62, 155, 70);
        for (const auto* label : {L"Area patrol", L"Great-circle cruise"})
            SendDlgItemMessageW(dialog, Mode, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label));
        constexpr const wchar_t* labels[]{L"Patrol radius (m)", L"Cruise speed (m/s)",
            L"Return speed (m/s)", L"Controlled speed (m/s)"};
        for (int i = 0; i < 4; ++i) {
            add(dialog, L"STATIC", labels[i], 0, -1, 10, 92 + i * 25, 155, 12);
            const auto edit = add(dialog, L"EDIT", L"", WS_TABSTOP | WS_BORDER | ES_AUTOHSCROLL,
                Radius + i, 175, 88 + i * 25, 155, 16);
            SendMessageW(edit, EM_SETLIMITTEXT, 30, 0);
        }
        add(dialog, L"STATIC", L"Controlled Invoker: Numpad + / - multiplies / divides base speed by 1.25; Shift temporarily multiplies it by 3.",
            0, -1, 10, 192, 320, 25);
        add(dialog, L"STATIC", L"Area patrol stays within the radius. Great-circle cruise follows a fixed plane through the planet center. Both follow the terrain. Camera follow keeps cruising. WASD takes manual control; Esc returns to the route before cruise resumes. A speed of 0 pauses cruise or return.",
            0, -1, 10, 222, 320, 45);
        add(dialog, L"STATIC", L"", 0, Validation, 10, 270, 320, 25);
        add(dialog, L"BUTTON", L"Defaults", WS_TABSTOP | BS_PUSHBUTTON, Defaults, 10, 301, 60, 16);
        add(dialog, L"BUTTON", L"Apply", WS_TABSTOP | BS_DEFPUSHBUTTON, IDOK, 200, 301, 60, 16);
        add(dialog, L"BUTTON", L"Cancel", WS_TABSTOP | BS_PUSHBUTTON, IDCANCEL, 270, 301, 60, 16);
        SendMessageW(dialog, DM_SETDEFID, IDOK, 0);
        populate(dialog, reinterpret_cast<Editor*>(l)->settings);
        return TRUE;
    }
    if (message == WM_CLOSE) { EndDialog(dialog, IDCANCEL); return TRUE; }
    if (message != WM_COMMAND) return FALSE;
    const int command = LOWORD(w);
    if (command == IDCANCEL) { EndDialog(dialog, IDCANCEL); return TRUE; }
    if (command == Defaults) { populate(dialog, InvokerSettings{}); return TRUE; }
    if (command == IDOK) {
        InvokerSettings settings;
        if (read(dialog, settings)) {
            reinterpret_cast<Editor*>(GetWindowLongPtrW(dialog, DWLP_USER))->settings = settings;
            EndDialog(dialog, IDOK);
        }
        return TRUE;
    }
    if (HIWORD(w) == EN_CHANGE || HIWORD(w) == CBN_SELCHANGE) { refresh(dialog); return TRUE; }
    return FALSE;
}

} // namespace

bool editInvokerSettings(HWND owner, InvokerSettings& settings) {
    const struct alignas(DWORD) {
        DLGTEMPLATE dialog;
        WORD menu, windowClass, title, pointSize;
        wchar_t font[9];
    } definition{{WS_POPUP | WS_CAPTION | WS_SYSMENU | DS_MODALFRAME | DS_CENTER | DS_SETFONT,
                   0, 0, 0, 0, 340, 329}, 0, 0, 0, 9, L"Segoe UI"};
    Editor editor{settings};
    const auto result = DialogBoxIndirectParamW(GetModuleHandleW(nullptr), &definition.dialog, owner,
        procedure, reinterpret_cast<LPARAM>(&editor));
    if (result == -1) throw std::runtime_error("Create Invoker settings dialog failed");
    if (result != IDOK) return false;
    settings = editor.settings;
    return true;
}

} // namespace voxel
