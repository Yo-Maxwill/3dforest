/****************************************************************************
** Meta object code from reading C++ file 'gui.h'
**
** Created: Thu 19. Mar 14:14:58 2015
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "gui.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'gui.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.4. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_InputDialog[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      13,   12,   12,   12, 0x08,
      18,   12,   12,   12, 0x08,
      36,   12,   12,   12, 0x08,
      57,   12,   12,   12, 0x08,
      82,   12,   12,   12, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_InputDialog[] = {
    "InputDialog\0\0ok()\0validate(QString)\0"
    "validateInt(QString)\0validateOutput1(QString)\0"
    "validateOutput2(QString)\0"
};

void InputDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        InputDialog *_t = static_cast<InputDialog *>(_o);
        switch (_id) {
        case 0: _t->ok(); break;
        case 1: _t->validate((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->validateInt((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->validateOutput1((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->validateOutput2((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData InputDialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject InputDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_InputDialog,
      qt_meta_data_InputDialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &InputDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *InputDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *InputDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_InputDialog))
        return static_cast<void*>(const_cast< InputDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int InputDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    }
    return _id;
}
static const uint qt_meta_data_ExportAttr[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x08,
      35,   11,   11,   11, 0x08,
      40,   11,   11,   11, 0x08,
      66,   58,   11,   11, 0x08,
      88,   58,   11,   11, 0x08,
     108,   58,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_ExportAttr[] = {
    "ExportAttr\0\0setExistingDirectory()\0"
    "ok()\0validate(QString)\0checked\0"
    "other_Separator(bool)\0all_attributes(int)\0"
    "all_attr(int)\0"
};

void ExportAttr::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ExportAttr *_t = static_cast<ExportAttr *>(_o);
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

const QMetaObjectExtraData ExportAttr::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ExportAttr::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_ExportAttr,
      qt_meta_data_ExportAttr, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ExportAttr::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ExportAttr::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ExportAttr::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ExportAttr))
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
    }
    return _id;
}
static const uint qt_meta_data_ProjImport[] = {

 // content:
       6,       // revision
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

static const char qt_meta_stringdata_ProjImport[] = {
    "ProjImport\0"
};

void ProjImport::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObjectExtraData ProjImport::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ProjImport::staticMetaObject = {
    { &QWizard::staticMetaObject, qt_meta_stringdata_ProjImport,
      qt_meta_data_ProjImport, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ProjImport::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ProjImport::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ProjImport::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ProjImport))
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
static const uint qt_meta_data_IntroPage[] = {

 // content:
       6,       // revision
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

static const char qt_meta_stringdata_IntroPage[] = {
    "IntroPage\0"
};

void IntroPage::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObjectExtraData IntroPage::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject IntroPage::staticMetaObject = {
    { &QWizardPage::staticMetaObject, qt_meta_stringdata_IntroPage,
      qt_meta_data_IntroPage, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &IntroPage::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *IntroPage::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *IntroPage::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_IntroPage))
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
static const uint qt_meta_data_NewProjectPage[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      16,   15,   15,   15, 0x08,
      44,   39,   15,   15, 0x08,
      63,   39,   15,   15, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_NewProjectPage[] = {
    "NewProjectPage\0\0setExistingDirectory()\0"
    "name\0nameCheck(QString)\0pathCheck(QString)\0"
};

void NewProjectPage::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        NewProjectPage *_t = static_cast<NewProjectPage *>(_o);
        switch (_id) {
        case 0: _t->setExistingDirectory(); break;
        case 1: _t->nameCheck((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 2: _t->pathCheck((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData NewProjectPage::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject NewProjectPage::staticMetaObject = {
    { &QWizardPage::staticMetaObject, qt_meta_stringdata_NewProjectPage,
      qt_meta_data_NewProjectPage, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &NewProjectPage::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *NewProjectPage::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *NewProjectPage::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_NewProjectPage))
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
    }
    return _id;
}
static const uint qt_meta_data_TransformPage[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      15,   14,   14,   14, 0x08,
      36,   30,   14,   14, 0x08,
      53,   30,   14,   14, 0x08,
      73,   67,   14,   14, 0x08,
      87,   14,   14,   14, 0x08,
     106,   14,   14,   14, 0x08,
     128,   14,   14,   14, 0x08,
     150,   14,   14,   14, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_TransformPage[] = {
    "TransformPage\0\0openProjects()\0check\0"
    "selectProj(bool)\0newProj(bool)\0index\0"
    "selected(int)\0nameCheck(QString)\0"
    "numberXCheck(QString)\0numberYCheck(QString)\0"
    "numberZCheck(QString)\0"
};

void TransformPage::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        TransformPage *_t = static_cast<TransformPage *>(_o);
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

const QMetaObjectExtraData TransformPage::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject TransformPage::staticMetaObject = {
    { &QWizardPage::staticMetaObject, qt_meta_stringdata_TransformPage,
      qt_meta_data_TransformPage, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &TransformPage::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *TransformPage::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *TransformPage::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_TransformPage))
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
    }
    return _id;
}
static const uint qt_meta_data_ImportPage[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x08,
      30,   11,   11,   11, 0x08,
      48,   43,   11,   11, 0x08,
      67,   43,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_ImportPage[] = {
    "ImportPage\0\0setNewDirectory()\0"
    "setOldFile()\0name\0nameCheck(QString)\0"
    "pathCheck(QString)\0"
};

void ImportPage::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        ImportPage *_t = static_cast<ImportPage *>(_o);
        switch (_id) {
        case 0: _t->setNewDirectory(); break;
        case 1: _t->setOldFile(); break;
        case 2: _t->nameCheck((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 3: _t->pathCheck((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData ImportPage::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject ImportPage::staticMetaObject = {
    { &QWizardPage::staticMetaObject, qt_meta_stringdata_ImportPage,
      qt_meta_data_ImportPage, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &ImportPage::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *ImportPage::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *ImportPage::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_ImportPage))
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
    }
    return _id;
}
static const uint qt_meta_data_FinalPage[] = {

 // content:
       6,       // revision
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

static const char qt_meta_stringdata_FinalPage[] = {
    "FinalPage\0"
};

void FinalPage::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObjectExtraData FinalPage::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject FinalPage::staticMetaObject = {
    { &QWizardPage::staticMetaObject, qt_meta_stringdata_FinalPage,
      qt_meta_data_FinalPage, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &FinalPage::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *FinalPage::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *FinalPage::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_FinalPage))
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
static const uint qt_meta_data_MyTree[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: signature, parameters, type, tag, flags
       8,    7,    7,    7, 0x05,
      27,    7,    7,    7, 0x05,
      47,    7,    7,    7, 0x05,
      67,    7,    7,    7, 0x05,
      86,    7,    7,    7, 0x05,
     110,    7,    7,    7, 0x05,

 // slots: signature, parameters, type, tag, flags
     129,  125,    7,    7, 0x08,
     160,  153,    7,    7, 0x08,
     200,  195,    7,    7, 0x08,
     222,  195,    7,    7, 0x08,
     239,  195,    7,    7, 0x08,
     261,  195,    7,    7, 0x08,
     278,    7,    7,    7, 0x08,
     286,    7,    7,    7, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MyTree[] = {
    "MyTree\0\0checkedON(QString)\0"
    "checkedOFF(QString)\0deleteItem(QString)\0"
    "colorItem(QString)\0colorItemField(QString)\0"
    "psize(QString)\0pos\0showContextMenu(QPoint)\0"
    "item,i\0onItemChange(QTreeWidgetItem*,int)\0"
    "name\0onDeleteItem(QString)\0onColor(QString)\0"
    "onColorField(QString)\0onPsize(QString)\0"
    "allON()\0allOFF()\0"
};

void MyTree::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MyTree *_t = static_cast<MyTree *>(_o);
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
    }
}

const QMetaObjectExtraData MyTree::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject MyTree::staticMetaObject = {
    { &QTreeWidget::staticMetaObject, qt_meta_stringdata_MyTree,
      qt_meta_data_MyTree, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MyTree::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MyTree::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MyTree::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MyTree))
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
    }
    return _id;
}

// SIGNAL 0
void MyTree::checkedON(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void MyTree::checkedOFF(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void MyTree::deleteItem(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void MyTree::colorItem(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void MyTree::colorItemField(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 4, _a);
}

// SIGNAL 5
void MyTree::psize(QString _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_END_MOC_NAMESPACE
