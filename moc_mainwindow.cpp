/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Wed 6. Aug 17:04:50 2014
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
static const uint qt_meta_data_PlusDialog[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x05,
      35,   11,   11,   11, 0x05,

 // slots: signature, parameters, type, tag, flags
      45,   11,   11,   11, 0x08,
      54,   11,   11,   11, 0x08,
      63,   11,   11,   11, 0x08,
      82,   11,   11,   11, 0x08,
     101,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_PlusDialog[] = {
    "PlusDialog\0\0pluscloud(QStringList)\0"
    "rejecte()\0accept()\0reject()\0"
    "setCloud1(QString)\0setCloud2(QString)\0"
    "setCloud3(QString)\0"
};

void PlusDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PlusDialog *_t = static_cast<PlusDialog *>(_o);
        switch (_id) {
        case 0: _t->pluscloud((*reinterpret_cast< QStringList(*)>(_a[1]))); break;
        case 1: _t->rejecte(); break;
        case 2: _t->accept(); break;
        case 3: _t->reject(); break;
        case 4: _t->setCloud1((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 5: _t->setCloud2((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 6: _t->setCloud3((*reinterpret_cast< QString(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData PlusDialog::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PlusDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_PlusDialog,
      qt_meta_data_PlusDialog, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PlusDialog::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PlusDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PlusDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PlusDialog))
        return static_cast<void*>(const_cast< PlusDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int PlusDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void PlusDialog::pluscloud(QStringList _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void PlusDialog::rejecte()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}
static const uint qt_meta_data_MyTree[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
       8,    7,    7,    7, 0x05,
      27,    7,    7,    7, 0x05,
      47,    7,    7,    7, 0x05,
      67,    7,    7,    7, 0x05,

 // slots: signature, parameters, type, tag, flags
      90,   86,    7,    7, 0x08,
     121,  114,    7,    7, 0x08,
     161,  156,    7,    7, 0x08,
     183,  156,    7,    7, 0x08,
     200,    7,    7,    7, 0x08,
     208,    7,    7,    7, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MyTree[] = {
    "MyTree\0\0checkedON(QString)\0"
    "checkedOFF(QString)\0deleteItem(QString)\0"
    "colorItem(QString)\0pos\0showContextMenu(QPoint)\0"
    "item,i\0onItemChange(QTreeWidgetItem*,int)\0"
    "name\0onDeleteItem(QString)\0onColor(QString)\0"
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
        case 4: _t->showContextMenu((*reinterpret_cast< const QPoint(*)>(_a[1]))); break;
        case 5: _t->onItemChange((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 6: _t->onDeleteItem((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->onColor((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 8: _t->allON(); break;
        case 9: _t->allOFF(); break;
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
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
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
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      51,   14, // methods
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
     361,   11,   11,   11, 0x08,
     375,   11,   11,   11, 0x08,
     393,   11,   11,   11, 0x08,
     411,   11,   11,   11, 0x08,
     431,  425,   11,   11, 0x08,
     456,   11,   11,   11, 0x08,
     468,   11,   11,   11, 0x08,
     516,  480,   11,   11, 0x08,
     641,   11,   11,   11, 0x08,
     654,   11,   11,   11, 0x08,
     669,   11,   11,   11, 0x08,
     688,   11,   11,   11, 0x08,
     704,   11,   11,   11, 0x08,
     724,   11,   11,   11, 0x08,
     748,  742,  738,   11, 0x08,
     790,   11,   11,   11, 0x08,
     796,   11,   11,   11, 0x08,
     805,   11,   11,   11, 0x08,
     816,   11,   11,   11, 0x08,
     831,   11,   11,   11, 0x08,
     850,   11,   11,   11, 0x08,
     861,   11,   11,   11, 0x08,
     876,   11,   11,   11, 0x08,
     885,   11,   11,   11, 0x08,
     900,   11,   11,   11, 0x08,
     911,   11,   11,   11, 0x08,
     929,  923,   11,   11, 0x08,
     952,   11,   11,   11, 0x08,
     963,   11,   11,   11, 0x08,
     981,   11,   11,   11, 0x08,
     989,   11,   11,   11, 0x08,
    1008, 1003,   11,   11, 0x08,
    1027, 1003,   11,   11, 0x08,
    1046, 1003,   11,   11, 0x08,
    1067, 1003,   11,   11, 0x08,
    1088, 1003,   11,   11, 0x08,

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
    "importTerrainFile()\0importCloud()\0"
    "importVegeCloud()\0importTreeCloud()\0"
    "exportCloud()\0event\0closeEvent(QCloseEvent*)\0"
    "voxelgrid()\0voxelstat()\0"
    "res,input,output_vege,output_ground\0"
    "octree(float,pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl:"
    ":PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr)\0"
    "octreeSlot()\0manualAdjust()\0"
    "manualAdjustStop()\0treeAtributes()\0"
    "treeAtributesRead()\0cylinderSeg()\0int\0"
    "input\0dbh(pcl::PointCloud<pcl::PointXYZI>::Ptr)\0"
    "dbh()\0height()\0position()\0manualSelect()\0"
    "manualSelectStop()\0treeEdit()\0"
    "treeEditStop()\0lenght()\0lenghtExport()\0"
    "seg_dist()\0plusCloud()\0names\0"
    "plusCloud(QStringList)\0voxelize()\0"
    "backgroundColor()\0about()\0removeCloud()\0"
    "name\0dispCloud(QString)\0showCLoud(QString)\0"
    "removeCloud(QString)\0deleteCloud(QString)\0"
    "colorCloud(QString)\0"
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
        case 14: _t->importTerrainFile(); break;
        case 15: _t->importCloud(); break;
        case 16: _t->importVegeCloud(); break;
        case 17: _t->importTreeCloud(); break;
        case 18: _t->exportCloud(); break;
        case 19: _t->closeEvent((*reinterpret_cast< QCloseEvent*(*)>(_a[1]))); break;
        case 20: _t->voxelgrid(); break;
        case 21: _t->voxelstat(); break;
        case 22: _t->octree((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< pcl::PointCloud<pcl::PointXYZI>::Ptr(*)>(_a[2])),(*reinterpret_cast< pcl::PointCloud<pcl::PointXYZI>::Ptr(*)>(_a[3])),(*reinterpret_cast< pcl::PointCloud<pcl::PointXYZI>::Ptr(*)>(_a[4]))); break;
        case 23: _t->octreeSlot(); break;
        case 24: _t->manualAdjust(); break;
        case 25: _t->manualAdjustStop(); break;
        case 26: _t->treeAtributes(); break;
        case 27: _t->treeAtributesRead(); break;
        case 28: _t->cylinderSeg(); break;
        case 29: { int _r = _t->dbh((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZI>::Ptr(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 30: _t->dbh(); break;
        case 31: _t->height(); break;
        case 32: _t->position(); break;
        case 33: _t->manualSelect(); break;
        case 34: _t->manualSelectStop(); break;
        case 35: _t->treeEdit(); break;
        case 36: _t->treeEditStop(); break;
        case 37: _t->lenght(); break;
        case 38: _t->lenghtExport(); break;
        case 39: _t->seg_dist(); break;
        case 40: _t->plusCloud(); break;
        case 41: _t->plusCloud((*reinterpret_cast< QStringList(*)>(_a[1]))); break;
        case 42: _t->voxelize(); break;
        case 43: _t->backgroundColor(); break;
        case 44: _t->about(); break;
        case 45: _t->removeCloud(); break;
        case 46: _t->dispCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 47: _t->showCLoud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 48: _t->removeCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 49: _t->deleteCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 50: _t->colorCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
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
        if (_id < 51)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 51;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
