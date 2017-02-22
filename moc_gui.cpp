/****************************************************************************
** Meta object code from reading C++ file 'gui.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "gui.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'gui.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_InputDialog_t {
    QByteArrayData data[15];
    char stringdata0[165];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_InputDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_InputDialog_t qt_meta_stringdata_InputDialog = {
    {
QT_MOC_LITERAL(0, 0, 11), // "InputDialog"
QT_MOC_LITERAL(1, 12, 12), // "inputCloud_1"
QT_MOC_LITERAL(2, 25, 0), // ""
QT_MOC_LITERAL(3, 26, 12), // "inputCloud_2"
QT_MOC_LITERAL(4, 39, 8), // "inputINT"
QT_MOC_LITERAL(5, 48, 2), // "ok"
QT_MOC_LITERAL(6, 51, 8), // "validate"
QT_MOC_LITERAL(7, 60, 11), // "validateInt"
QT_MOC_LITERAL(8, 72, 15), // "validateOutput1"
QT_MOC_LITERAL(9, 88, 15), // "validateOutput2"
QT_MOC_LITERAL(10, 104, 11), // "saveNewFile"
QT_MOC_LITERAL(11, 116, 11), // "saveIntoDir"
QT_MOC_LITERAL(12, 128, 6), // "getINT"
QT_MOC_LITERAL(13, 135, 14), // "getinputCloud1"
QT_MOC_LITERAL(14, 150, 14) // "getinputCloud2"

    },
    "InputDialog\0inputCloud_1\0\0inputCloud_2\0"
    "inputINT\0ok\0validate\0validateInt\0"
    "validateOutput1\0validateOutput2\0"
    "saveNewFile\0saveIntoDir\0getINT\0"
    "getinputCloud1\0getinputCloud2"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_InputDialog[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      13,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   79,    2, 0x06 /* Public */,
       3,    1,   82,    2, 0x06 /* Public */,
       4,    1,   85,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   88,    2, 0x08 /* Private */,
       6,    1,   89,    2, 0x08 /* Private */,
       7,    1,   92,    2, 0x08 /* Private */,
       8,    1,   95,    2, 0x08 /* Private */,
       9,    1,   98,    2, 0x08 /* Private */,
      10,    0,  101,    2, 0x08 /* Private */,
      11,    0,  102,    2, 0x08 /* Private */,
      12,    0,  103,    2, 0x0a /* Public */,
      13,    0,  104,    2, 0x0a /* Public */,
      14,    0,  105,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::Int,    2,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void InputDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        InputDialog *_t = static_cast<InputDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->inputCloud_1((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->inputCloud_2((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->inputINT((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->ok(); break;
        case 4: _t->validate((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->validateInt((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->validateOutput1((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->validateOutput2((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 8: _t->saveNewFile(); break;
        case 9: _t->saveIntoDir(); break;
        case 10: _t->getINT(); break;
        case 11: _t->getinputCloud1(); break;
        case 12: _t->getinputCloud2(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (InputDialog::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&InputDialog::inputCloud_1)) {
                *result = 0;
            }
        }
        {
            typedef void (InputDialog::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&InputDialog::inputCloud_2)) {
                *result = 1;
            }
        }
        {
            typedef void (InputDialog::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&InputDialog::inputINT)) {
                *result = 2;
            }
        }
    }
}

const QMetaObject InputDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_InputDialog.data,
      qt_meta_data_InputDialog,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *InputDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *InputDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_InputDialog.stringdata0))
        return static_cast<void*>(const_cast< InputDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int InputDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 13)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 13;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 13)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 13;
    }
    return _id;
}

// SIGNAL 0
void InputDialog::inputCloud_1(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void InputDialog::inputCloud_2(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void InputDialog::inputINT(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
struct qt_meta_stringdata_ExportAttr_t {
    QByteArrayData data[9];
    char stringdata0[93];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ExportAttr_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ExportAttr_t qt_meta_stringdata_ExportAttr = {
    {
QT_MOC_LITERAL(0, 0, 10), // "ExportAttr"
QT_MOC_LITERAL(1, 11, 20), // "setExistingDirectory"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 2), // "ok"
QT_MOC_LITERAL(4, 36, 8), // "validate"
QT_MOC_LITERAL(5, 45, 15), // "other_Separator"
QT_MOC_LITERAL(6, 61, 7), // "checked"
QT_MOC_LITERAL(7, 69, 14), // "all_attributes"
QT_MOC_LITERAL(8, 84, 8) // "all_attr"

    },
    "ExportAttr\0setExistingDirectory\0\0ok\0"
    "validate\0other_Separator\0checked\0"
    "all_attributes\0all_attr"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ExportAttr[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   44,    2, 0x08 /* Private */,
       3,    0,   45,    2, 0x08 /* Private */,
       4,    1,   46,    2, 0x08 /* Private */,
       5,    1,   49,    2, 0x08 /* Private */,
       7,    1,   52,    2, 0x08 /* Private */,
       8,    1,   55,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void, QMetaType::Int,    6,
    QMetaType::Void, QMetaType::Int,    6,

       0        // eod
};

void ExportAttr::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ExportAttr *_t = static_cast<ExportAttr *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setExistingDirectory(); break;
        case 1: _t->ok(); break;
        case 2: _t->validate((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->other_Separator((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->all_attributes((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->all_attr((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject ExportAttr::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_ExportAttr.data,
      qt_meta_data_ExportAttr,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ExportAttr::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ExportAttr::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ExportAttr.stringdata0))
        return static_cast<void*>(const_cast< ExportAttr*>(this));
    return QDialog::qt_metacast(_clname);
}

int ExportAttr::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
struct qt_meta_stringdata_ProjImport_t {
    QByteArrayData data[1];
    char stringdata0[11];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ProjImport_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ProjImport_t qt_meta_stringdata_ProjImport = {
    {
QT_MOC_LITERAL(0, 0, 10) // "ProjImport"

    },
    "ProjImport"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ProjImport[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void ProjImport::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject ProjImport::staticMetaObject = {
    { &QWizard::staticMetaObject, qt_meta_stringdata_ProjImport.data,
      qt_meta_data_ProjImport,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ProjImport::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ProjImport::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ProjImport.stringdata0))
        return static_cast<void*>(const_cast< ProjImport*>(this));
    return QWizard::qt_metacast(_clname);
}

int ProjImport::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWizard::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
struct qt_meta_stringdata_IntroPage_t {
    QByteArrayData data[1];
    char stringdata0[10];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_IntroPage_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_IntroPage_t qt_meta_stringdata_IntroPage = {
    {
QT_MOC_LITERAL(0, 0, 9) // "IntroPage"

    },
    "IntroPage"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_IntroPage[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void IntroPage::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject IntroPage::staticMetaObject = {
    { &QWizardPage::staticMetaObject, qt_meta_stringdata_IntroPage.data,
      qt_meta_data_IntroPage,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *IntroPage::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *IntroPage::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_IntroPage.stringdata0))
        return static_cast<void*>(const_cast< IntroPage*>(this));
    return QWizardPage::qt_metacast(_clname);
}

int IntroPage::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWizardPage::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
struct qt_meta_stringdata_NewProjectPage_t {
    QByteArrayData data[6];
    char stringdata0[62];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_NewProjectPage_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_NewProjectPage_t qt_meta_stringdata_NewProjectPage = {
    {
QT_MOC_LITERAL(0, 0, 14), // "NewProjectPage"
QT_MOC_LITERAL(1, 15, 20), // "setExistingDirectory"
QT_MOC_LITERAL(2, 36, 0), // ""
QT_MOC_LITERAL(3, 37, 9), // "nameCheck"
QT_MOC_LITERAL(4, 47, 4), // "name"
QT_MOC_LITERAL(5, 52, 9) // "pathCheck"

    },
    "NewProjectPage\0setExistingDirectory\0"
    "\0nameCheck\0name\0pathCheck"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_NewProjectPage[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   29,    2, 0x08 /* Private */,
       3,    1,   30,    2, 0x08 /* Private */,
       5,    1,   33,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    4,
    QMetaType::Void, QMetaType::QString,    4,

       0        // eod
};

void NewProjectPage::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        NewProjectPage *_t = static_cast<NewProjectPage *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setExistingDirectory(); break;
        case 1: _t->nameCheck((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->pathCheck((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject NewProjectPage::staticMetaObject = {
    { &QWizardPage::staticMetaObject, qt_meta_stringdata_NewProjectPage.data,
      qt_meta_data_NewProjectPage,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *NewProjectPage::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *NewProjectPage::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_NewProjectPage.stringdata0))
        return static_cast<void*>(const_cast< NewProjectPage*>(this));
    return QWizardPage::qt_metacast(_clname);
}

int NewProjectPage::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWizardPage::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}
struct qt_meta_stringdata_TransformPage_t {
    QByteArrayData data[12];
    char stringdata0[117];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_TransformPage_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_TransformPage_t qt_meta_stringdata_TransformPage = {
    {
QT_MOC_LITERAL(0, 0, 13), // "TransformPage"
QT_MOC_LITERAL(1, 14, 12), // "openProjects"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 10), // "selectProj"
QT_MOC_LITERAL(4, 39, 5), // "check"
QT_MOC_LITERAL(5, 45, 7), // "newProj"
QT_MOC_LITERAL(6, 53, 8), // "selected"
QT_MOC_LITERAL(7, 62, 5), // "index"
QT_MOC_LITERAL(8, 68, 9), // "nameCheck"
QT_MOC_LITERAL(9, 78, 12), // "numberXCheck"
QT_MOC_LITERAL(10, 91, 12), // "numberYCheck"
QT_MOC_LITERAL(11, 104, 12) // "numberZCheck"

    },
    "TransformPage\0openProjects\0\0selectProj\0"
    "check\0newProj\0selected\0index\0nameCheck\0"
    "numberXCheck\0numberYCheck\0numberZCheck"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_TransformPage[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   54,    2, 0x08 /* Private */,
       3,    1,   55,    2, 0x08 /* Private */,
       5,    1,   58,    2, 0x08 /* Private */,
       6,    1,   61,    2, 0x08 /* Private */,
       8,    1,   64,    2, 0x08 /* Private */,
       9,    1,   67,    2, 0x08 /* Private */,
      10,    1,   70,    2, 0x08 /* Private */,
      11,    1,   73,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Bool,    4,
    QMetaType::Void, QMetaType::Int,    7,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,

       0        // eod
};

void TransformPage::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        TransformPage *_t = static_cast<TransformPage *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->openProjects(); break;
        case 1: _t->selectProj((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->newProj((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->selected((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->nameCheck((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->numberXCheck((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->numberYCheck((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->numberZCheck((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject TransformPage::staticMetaObject = {
    { &QWizardPage::staticMetaObject, qt_meta_stringdata_TransformPage.data,
      qt_meta_data_TransformPage,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *TransformPage::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *TransformPage::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_TransformPage.stringdata0))
        return static_cast<void*>(const_cast< TransformPage*>(this));
    return QWizardPage::qt_metacast(_clname);
}

int TransformPage::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWizardPage::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}
struct qt_meta_stringdata_ImportPage_t {
    QByteArrayData data[7];
    char stringdata0[64];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ImportPage_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ImportPage_t qt_meta_stringdata_ImportPage = {
    {
QT_MOC_LITERAL(0, 0, 10), // "ImportPage"
QT_MOC_LITERAL(1, 11, 15), // "setNewDirectory"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 10), // "setOldFile"
QT_MOC_LITERAL(4, 39, 9), // "nameCheck"
QT_MOC_LITERAL(5, 49, 4), // "name"
QT_MOC_LITERAL(6, 54, 9) // "pathCheck"

    },
    "ImportPage\0setNewDirectory\0\0setOldFile\0"
    "nameCheck\0name\0pathCheck"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ImportPage[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   34,    2, 0x08 /* Private */,
       3,    0,   35,    2, 0x08 /* Private */,
       4,    1,   36,    2, 0x08 /* Private */,
       6,    1,   39,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    5,
    QMetaType::Void, QMetaType::QString,    5,

       0        // eod
};

void ImportPage::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ImportPage *_t = static_cast<ImportPage *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setNewDirectory(); break;
        case 1: _t->setOldFile(); break;
        case 2: _t->nameCheck((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->pathCheck((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject ImportPage::staticMetaObject = {
    { &QWizardPage::staticMetaObject, qt_meta_stringdata_ImportPage.data,
      qt_meta_data_ImportPage,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ImportPage::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ImportPage::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ImportPage.stringdata0))
        return static_cast<void*>(const_cast< ImportPage*>(this));
    return QWizardPage::qt_metacast(_clname);
}

int ImportPage::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWizardPage::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}
struct qt_meta_stringdata_FinalPage_t {
    QByteArrayData data[1];
    char stringdata0[10];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_FinalPage_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_FinalPage_t qt_meta_stringdata_FinalPage = {
    {
QT_MOC_LITERAL(0, 0, 9) // "FinalPage"

    },
    "FinalPage"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_FinalPage[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void FinalPage::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject FinalPage::staticMetaObject = {
    { &QWizardPage::staticMetaObject, qt_meta_stringdata_FinalPage.data,
      qt_meta_data_FinalPage,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *FinalPage::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *FinalPage::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_FinalPage.stringdata0))
        return static_cast<void*>(const_cast< FinalPage*>(this));
    return QWizardPage::qt_metacast(_clname);
}

int FinalPage::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWizardPage::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
struct qt_meta_stringdata_MyTree_t {
    QByteArrayData data[21];
    char stringdata0[188];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MyTree_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MyTree_t qt_meta_stringdata_MyTree = {
    {
QT_MOC_LITERAL(0, 0, 6), // "MyTree"
QT_MOC_LITERAL(1, 7, 9), // "checkedON"
QT_MOC_LITERAL(2, 17, 0), // ""
QT_MOC_LITERAL(3, 18, 10), // "checkedOFF"
QT_MOC_LITERAL(4, 29, 10), // "deleteItem"
QT_MOC_LITERAL(5, 40, 9), // "colorItem"
QT_MOC_LITERAL(6, 50, 14), // "colorItemField"
QT_MOC_LITERAL(7, 65, 5), // "psize"
QT_MOC_LITERAL(8, 71, 15), // "showContextMenu"
QT_MOC_LITERAL(9, 87, 3), // "pos"
QT_MOC_LITERAL(10, 91, 12), // "onItemChange"
QT_MOC_LITERAL(11, 104, 16), // "QTreeWidgetItem*"
QT_MOC_LITERAL(12, 121, 4), // "item"
QT_MOC_LITERAL(13, 126, 1), // "i"
QT_MOC_LITERAL(14, 128, 12), // "onDeleteItem"
QT_MOC_LITERAL(15, 141, 4), // "name"
QT_MOC_LITERAL(16, 146, 7), // "onColor"
QT_MOC_LITERAL(17, 154, 12), // "onColorField"
QT_MOC_LITERAL(18, 167, 7), // "onPsize"
QT_MOC_LITERAL(19, 175, 5), // "allON"
QT_MOC_LITERAL(20, 181, 6) // "allOFF"

    },
    "MyTree\0checkedON\0\0checkedOFF\0deleteItem\0"
    "colorItem\0colorItemField\0psize\0"
    "showContextMenu\0pos\0onItemChange\0"
    "QTreeWidgetItem*\0item\0i\0onDeleteItem\0"
    "name\0onColor\0onColorField\0onPsize\0"
    "allON\0allOFF"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MyTree[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   84,    2, 0x06 /* Public */,
       3,    1,   87,    2, 0x06 /* Public */,
       4,    1,   90,    2, 0x06 /* Public */,
       5,    1,   93,    2, 0x06 /* Public */,
       6,    1,   96,    2, 0x06 /* Public */,
       7,    1,   99,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       8,    1,  102,    2, 0x08 /* Private */,
      10,    2,  105,    2, 0x08 /* Private */,
      14,    1,  110,    2, 0x08 /* Private */,
      16,    1,  113,    2, 0x08 /* Private */,
      17,    1,  116,    2, 0x08 /* Private */,
      18,    1,  119,    2, 0x08 /* Private */,
      19,    0,  122,    2, 0x08 /* Private */,
      20,    0,  123,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::QString,    2,

 // slots: parameters
    QMetaType::Void, QMetaType::QPoint,    9,
    QMetaType::Void, 0x80000000 | 11, QMetaType::Int,   12,   13,
    QMetaType::Void, QMetaType::QString,   15,
    QMetaType::Void, QMetaType::QString,   15,
    QMetaType::Void, QMetaType::QString,   15,
    QMetaType::Void, QMetaType::QString,   15,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MyTree::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MyTree *_t = static_cast<MyTree *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->checkedON((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->checkedOFF((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->deleteItem((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->colorItem((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->colorItemField((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->psize((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->showContextMenu((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 7: _t->onItemChange((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 8: _t->onDeleteItem((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 9: _t->onColor((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 10: _t->onColorField((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 11: _t->onPsize((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 12: _t->allON(); break;
        case 13: _t->allOFF(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (MyTree::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MyTree::checkedON)) {
                *result = 0;
            }
        }
        {
            typedef void (MyTree::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MyTree::checkedOFF)) {
                *result = 1;
            }
        }
        {
            typedef void (MyTree::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MyTree::deleteItem)) {
                *result = 2;
            }
        }
        {
            typedef void (MyTree::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MyTree::colorItem)) {
                *result = 3;
            }
        }
        {
            typedef void (MyTree::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MyTree::colorItemField)) {
                *result = 4;
            }
        }
        {
            typedef void (MyTree::*_t)(QString );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MyTree::psize)) {
                *result = 5;
            }
        }
    }
}

const QMetaObject MyTree::staticMetaObject = {
    { &QTreeWidget::staticMetaObject, qt_meta_stringdata_MyTree.data,
      qt_meta_data_MyTree,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MyTree::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MyTree::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MyTree.stringdata0))
        return static_cast<void*>(const_cast< MyTree*>(this));
    return QTreeWidget::qt_metacast(_clname);
}

int MyTree::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QTreeWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 14)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 14;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 14)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 14;
    }
    return _id;
}

// SIGNAL 0
void MyTree::checkedON(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MyTree::checkedOFF(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void MyTree::deleteItem(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void MyTree::colorItem(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void MyTree::colorItemField(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void MyTree::psize(QString _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
struct qt_meta_stringdata_ExportCrownAttr_t {
    QByteArrayData data[9];
    char stringdata0[98];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ExportCrownAttr_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ExportCrownAttr_t qt_meta_stringdata_ExportCrownAttr = {
    {
QT_MOC_LITERAL(0, 0, 15), // "ExportCrownAttr"
QT_MOC_LITERAL(1, 16, 20), // "setExistingDirectory"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 2), // "ok"
QT_MOC_LITERAL(4, 41, 8), // "validate"
QT_MOC_LITERAL(5, 50, 15), // "other_Separator"
QT_MOC_LITERAL(6, 66, 7), // "checked"
QT_MOC_LITERAL(7, 74, 14), // "all_attributes"
QT_MOC_LITERAL(8, 89, 8) // "all_attr"

    },
    "ExportCrownAttr\0setExistingDirectory\0"
    "\0ok\0validate\0other_Separator\0checked\0"
    "all_attributes\0all_attr"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ExportCrownAttr[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   44,    2, 0x08 /* Private */,
       3,    0,   45,    2, 0x08 /* Private */,
       4,    1,   46,    2, 0x08 /* Private */,
       5,    1,   49,    2, 0x08 /* Private */,
       7,    1,   52,    2, 0x08 /* Private */,
       8,    1,   55,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    2,
    QMetaType::Void, QMetaType::Bool,    6,
    QMetaType::Void, QMetaType::Int,    6,
    QMetaType::Void, QMetaType::Int,    6,

       0        // eod
};

void ExportCrownAttr::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ExportCrownAttr *_t = static_cast<ExportCrownAttr *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setExistingDirectory(); break;
        case 1: _t->ok(); break;
        case 2: _t->validate((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->other_Separator((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->all_attributes((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->all_attr((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject ExportCrownAttr::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_ExportCrownAttr.data,
      qt_meta_data_ExportCrownAttr,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ExportCrownAttr::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ExportCrownAttr::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ExportCrownAttr.stringdata0))
        return static_cast<void*>(const_cast< ExportCrownAttr*>(this));
    return QDialog::qt_metacast(_clname);
}

int ExportCrownAttr::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
