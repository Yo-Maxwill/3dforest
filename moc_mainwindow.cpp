/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Fri 23. Jan 09:51:36 2015
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.4)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwindow.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
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
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      65,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x08,
      25,   11,   11,   11, 0x08,
      39,   11,   11,   11, 0x08,
      59,   54,   11,   11, 0x08,
      84,   54,   11,   11, 0x08,
     106,   54,   11,   11, 0x08,
     127,   54,   11,   11, 0x08,
     149,   54,   11,   11, 0x08,
     181,  172,   11,   11, 0x08,
     213,  172,   11,   11, 0x08,
     242,  172,   11,   11, 0x08,
     270,  172,   11,   11, 0x08,
     299,  172,   11,   11, 0x08,
     329,   11,   11,   11, 0x08,
     341,   11,   11,   11, 0x08,
     354,   11,   11,   11, 0x08,
     366,   11,   11,   11, 0x08,
     386,   11,   11,   11, 0x08,
     400,   11,   11,   11, 0x08,
     418,   11,   11,   11, 0x08,
     436,   11,   11,   11, 0x08,
     456,  450,   11,   11, 0x08,
     481,   11,   11,   11, 0x08,
     493,   11,   11,   11, 0x08,
     541,  505,   11,   11, 0x08,
     666,   11,   11,   11, 0x08,
     679,   11,   11,   11, 0x08,
     694,   11,   11,   11, 0x08,
     713,   11,   11,   11, 0x08,
     720,   11,   11,   11, 0x08,
     736,   11,   11,   11, 0x08,
     756,   11,   11,   11, 0x08,
     780,  774,  770,   11, 0x08,
     822,   11,   11,   11, 0x08,
     828,   11,   11,   11, 0x08,
     837,   11,   11,   11, 0x08,
     846,   11,   11,   11, 0x08,
     857,   11,   11,   11, 0x08,
     872,   11,   11,   11, 0x08,
     891,   11,   11,   11, 0x08,
     902,   11,   11,   11, 0x08,
     917,   11,   11,   11, 0x08,
     926,   11,   11,   11, 0x08,
     937,   11,   11,   11, 0x08,
     948,   11,   11,   11, 0x08,
     963,   11,   11,   11, 0x08,
     982,   11,   11,   11, 0x08,
     992,   11,   11,   11, 0x08,
    1008, 1004,   11,   11, 0x08,
    1051,   11,   11,   11, 0x08,
    1062,   11,   11,   11, 0x08,
    1080,   11,   11,   11, 0x08,
    1086,   11,   11,   11, 0x08,
    1104, 1093,   11,   11, 0x08,
    1126,   11,   11,   11, 0x08,
    1136,   11,   11,   11, 0x08,
    1147,   11,   11,   11, 0x08,
    1155,   11,   11,   11, 0x08,
    1174, 1169,   11,   11, 0x08,
    1193, 1169,   11,   11, 0x08,
    1212, 1169,   11,   11, 0x08,
    1233, 1169,   11,   11, 0x08,
    1254, 1169,   11,   11, 0x08,
    1274, 1169,   11,   11, 0x08,
    1299, 1169,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0newProject()\0openProject()\0"
    "closeProject()\0file\0openTerrainFile(QString)\0"
    "openVegeFile(QString)\0openOstFile(QString)\0"
    "openTreeFile(QString)\0openCloudFile(QString)\0"
    "file,col\0openTerrainFile(QString,QColor)\0"
    "openVegeFile(QString,QColor)\0"
    "openOstFile(QString,QColor)\0"
    "openTreeFile(QString,QColor)\0"
    "openCloudFile(QString,QColor)\0importtxt()\0"
    "importtxt2()\0importlas()\0importTerrainFile()\0"
    "importCloud()\0importVegeCloud()\0"
    "importTreeCloud()\0exportCloud()\0event\0"
    "closeEvent(QCloseEvent*)\0voxelgrid()\0"
    "voxelstat()\0res,input,output_vege,output_ground\0"
    "octree(float,pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl:"
    ":PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr)\0"
    "octreeSlot()\0manualAdjust()\0"
    "manualAdjustStop()\0undo()\0treeAtributes()\0"
    "treeAtributesRead()\0cylinderSeg()\0int\0"
    "input\0dbh(pcl::PointCloud<pcl::PointXYZI>::Ptr)\0"
    "dbh()\0dbhLSR()\0height()\0position()\0"
    "manualSelect()\0manualSelectStop()\0"
    "treeEdit()\0treeEditStop()\0lenght()\0"
    "seg_dist()\0skeleton()\0dbhCloudEdit()\0"
    "dbhCloudStopEdit()\0plysave()\0plusCloud()\0"
    ",,,\0plusCloud(QString,QString,QString,QString)\0"
    "voxelize()\0backgroundColor()\0IDW()\0"
    "clip()\0cl,cl2,res\0clip(Cloud,Cloud,int)\0"
    "clipped()\0clipStop()\0about()\0removeCloud()\0"
    "name\0dispCloud(QString)\0showCLoud(QString)\0"
    "removeCloud(QString)\0deleteCloud(QString)\0"
    "colorCloud(QString)\0colorCloudField(QString)\0"
    "PointSize(QString)\0"
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        MainWindow *_t = static_cast<MainWindow *>(_o);
        switch (_id) {
        case 0: _t->newProject(); break;
        case 1: _t->openProject(); break;
        case 2: _t->closeProject(); break;
        case 3: _t->openTerrainFile((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 4: _t->openVegeFile((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->openOstFile((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->openTreeFile((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->openCloudFile((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 8: _t->openTerrainFile((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QColor(*)>(_a[2]))); break;
        case 9: _t->openVegeFile((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QColor(*)>(_a[2]))); break;
        case 10: _t->openOstFile((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QColor(*)>(_a[2]))); break;
        case 11: _t->openTreeFile((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QColor(*)>(_a[2]))); break;
        case 12: _t->openCloudFile((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QColor(*)>(_a[2]))); break;
        case 13: _t->importtxt(); break;
        case 14: _t->importtxt2(); break;
        case 15: _t->importlas(); break;
        case 16: _t->importTerrainFile(); break;
        case 17: _t->importCloud(); break;
        case 18: _t->importVegeCloud(); break;
        case 19: _t->importTreeCloud(); break;
        case 20: _t->exportCloud(); break;
        case 21: _t->closeEvent((*reinterpret_cast< QCloseEvent*(*)>(_a[1]))); break;
        case 22: _t->voxelgrid(); break;
        case 23: _t->voxelstat(); break;
        case 24: _t->octree((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< pcl::PointCloud<pcl::PointXYZI>::Ptr(*)>(_a[2])),(*reinterpret_cast< pcl::PointCloud<pcl::PointXYZI>::Ptr(*)>(_a[3])),(*reinterpret_cast< pcl::PointCloud<pcl::PointXYZI>::Ptr(*)>(_a[4]))); break;
        case 25: _t->octreeSlot(); break;
        case 26: _t->manualAdjust(); break;
        case 27: _t->manualAdjustStop(); break;
        case 28: _t->undo(); break;
        case 29: _t->treeAtributes(); break;
        case 30: _t->treeAtributesRead(); break;
        case 31: _t->cylinderSeg(); break;
        case 32: { int _r = _t->dbh((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZI>::Ptr(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 33: _t->dbh(); break;
        case 34: _t->dbhLSR(); break;
        case 35: _t->height(); break;
        case 36: _t->position(); break;
        case 37: _t->manualSelect(); break;
        case 38: _t->manualSelectStop(); break;
        case 39: _t->treeEdit(); break;
        case 40: _t->treeEditStop(); break;
        case 41: _t->lenght(); break;
        case 42: _t->seg_dist(); break;
        case 43: _t->skeleton(); break;
        case 44: _t->dbhCloudEdit(); break;
        case 45: _t->dbhCloudStopEdit(); break;
        case 46: _t->plysave(); break;
        case 47: _t->plusCloud(); break;
        case 48: _t->plusCloud((*reinterpret_cast< QString(*)>(_a[1])),(*reinterpret_cast< QString(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 49: _t->voxelize(); break;
        case 50: _t->backgroundColor(); break;
        case 51: _t->IDW(); break;
        case 52: _t->clip(); break;
        case 53: _t->clip((*reinterpret_cast< Cloud(*)>(_a[1])),(*reinterpret_cast< Cloud(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3]))); break;
        case 54: _t->clipped(); break;
        case 55: _t->clipStop(); break;
        case 56: _t->about(); break;
        case 57: _t->removeCloud(); break;
        case 58: _t->dispCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 59: _t->showCLoud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 60: _t->removeCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 61: _t->deleteCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 62: _t->colorCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 63: _t->colorCloudField((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 64: _t->PointSize((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData MainWindow::staticMetaObjectExtraData = {
    0,  qt_static_metacall
};

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow,
      qt_meta_data_MainWindow, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 65)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 65;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
