/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created: Wed 18. Mar 08:14:46 2015
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
static const uint qt_meta_data_MainWindow[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      52,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      12,   11,   11,   11, 0x08,
      25,   11,   11,   11, 0x08,
      39,   11,   11,   11, 0x08,
      54,   11,   11,   11, 0x08,
      70,   11,   11,   11, 0x08,
      82,   11,   11,   11, 0x08,
      94,   11,   11,   11, 0x08,
     106,   11,   11,   11, 0x08,
     118,   11,   11,   11, 0x08,
     138,   11,   11,   11, 0x08,
     152,   11,   11,   11, 0x08,
     170,   11,   11,   11, 0x08,
     188,   11,   11,   11, 0x08,
     202,   11,   11,   11, 0x08,
     212,   11,   11,   11, 0x08,
     224,   11,   11,   11, 0x08,
     242,   11,   11,   11, 0x08,
     267,  261,   11,   11, 0x08,
     292,   11,   11,   11, 0x08,
     304,   11,   11,   11, 0x08,
     317,   11,   11,   11, 0x08,
     332,   11,   11,   11, 0x08,
     351,   11,   11,   11, 0x08,
     366,   11,   11,   11, 0x08,
     385,   11,   11,   11, 0x08,
     396,   11,   11,   11, 0x08,
     411,   11,   11,   11, 0x08,
     427,   11,   11,   11, 0x08,
     435,   11,   11,   11, 0x08,
     444,   11,   11,   11, 0x08,
     453,   11,   11,   11, 0x08,
     464,   11,   11,   11, 0x08,
     473,   11,   11,   11, 0x08,
     484,   11,   11,   11, 0x08,
     497,   11,   11,   11, 0x08,
     511,   11,   11,   11, 0x08,
     526,   11,   11,   11, 0x08,
     545,   11,   11,   11, 0x08,
     557,   11,   11,   11, 0x08,
     570,   11,   11,   11, 0x08,
     581,   11,   11,   11, 0x08,
     587,   11,   11,   11, 0x08,
     594,   11,   11,   11, 0x08,
     604,   11,   11,   11, 0x08,
     615,   11,   11,   11, 0x08,
     628,  623,   11,   11, 0x08,
     647,  623,   11,   11, 0x08,
     668,  623,   11,   11, 0x08,
     689,  623,   11,   11, 0x08,
     709,  623,   11,   11, 0x08,
     734,  623,   11,   11, 0x08,
     753,   11,   11,   11, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_MainWindow[] = {
    "MainWindow\0\0newProject()\0openProject()\0"
    "closeProject()\0importProject()\0"
    "importtxt()\0importlas()\0importpts()\0"
    "importptx()\0importTerrainFile()\0"
    "importCloud()\0importVegeCloud()\0"
    "importTreeCloud()\0exportCloud()\0"
    "plysave()\0exportPts()\0exportConvexTxt()\0"
    "exportConcaveTxt()\0event\0"
    "closeEvent(QCloseEvent*)\0voxelgrid()\0"
    "octreeSlot()\0manualAdjust()\0"
    "manualAdjustStop()\0manualSelect()\0"
    "manualSelectStop()\0treeEdit()\0"
    "treeEditStop()\0treeAtributes()\0dbhHT()\0"
    "dbhLSR()\0height()\0position()\0lenght()\0"
    "skeleton()\0convexhull()\0concavehull()\0"
    "dbhCloudEdit()\0dbhCloudStopEdit()\0"
    "plusCloud()\0minusCloud()\0voxelize()\0"
    "IDW()\0clip()\0clipped()\0clipStop()\0"
    "about()\0name\0dispCloud(QString)\0"
    "removeCloud(QString)\0deleteCloud(QString)\0"
    "colorCloud(QString)\0colorCloudField(QString)\0"
    "PointSize(QString)\0undo()\0"
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
        case 3: _t->importProject(); break;
        case 4: _t->importtxt(); break;
        case 5: _t->importlas(); break;
        case 6: _t->importpts(); break;
        case 7: _t->importptx(); break;
        case 8: _t->importTerrainFile(); break;
        case 9: _t->importCloud(); break;
        case 10: _t->importVegeCloud(); break;
        case 11: _t->importTreeCloud(); break;
        case 12: _t->exportCloud(); break;
        case 13: _t->plysave(); break;
        case 14: _t->exportPts(); break;
        case 15: _t->exportConvexTxt(); break;
        case 16: _t->exportConcaveTxt(); break;
        case 17: _t->closeEvent((*reinterpret_cast< QCloseEvent*(*)>(_a[1]))); break;
        case 18: _t->voxelgrid(); break;
        case 19: _t->octreeSlot(); break;
        case 20: _t->manualAdjust(); break;
        case 21: _t->manualAdjustStop(); break;
        case 22: _t->manualSelect(); break;
        case 23: _t->manualSelectStop(); break;
        case 24: _t->treeEdit(); break;
        case 25: _t->treeEditStop(); break;
        case 26: _t->treeAtributes(); break;
        case 27: _t->dbhHT(); break;
        case 28: _t->dbhLSR(); break;
        case 29: _t->height(); break;
        case 30: _t->position(); break;
        case 31: _t->lenght(); break;
        case 32: _t->skeleton(); break;
        case 33: _t->convexhull(); break;
        case 34: _t->concavehull(); break;
        case 35: _t->dbhCloudEdit(); break;
        case 36: _t->dbhCloudStopEdit(); break;
        case 37: _t->plusCloud(); break;
        case 38: _t->minusCloud(); break;
        case 39: _t->voxelize(); break;
        case 40: _t->IDW(); break;
        case 41: _t->clip(); break;
        case 42: _t->clipped(); break;
        case 43: _t->clipStop(); break;
        case 44: _t->about(); break;
        case 45: _t->dispCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 46: _t->removeCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 47: _t->deleteCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 48: _t->colorCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 49: _t->colorCloudField((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 50: _t->PointSize((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 51: _t->undo(); break;
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
        if (_id < 52)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 52;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
