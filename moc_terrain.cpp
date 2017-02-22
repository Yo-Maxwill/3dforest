/****************************************************************************
** Meta object code from reading C++ file 'terrain.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "terrain.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'terrain.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_OctreeTerrain_t {
    QByteArrayData data[22];
    char stringdata0[250];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_OctreeTerrain_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_OctreeTerrain_t qt_meta_stringdata_OctreeTerrain = {
    {
QT_MOC_LITERAL(0, 0, 13), // "OctreeTerrain"
QT_MOC_LITERAL(1, 14, 8), // "finished"
QT_MOC_LITERAL(2, 23, 0), // ""
QT_MOC_LITERAL(3, 24, 10), // "percentage"
QT_MOC_LITERAL(4, 35, 14), // "sendingTerrain"
QT_MOC_LITERAL(5, 50, 6), // "Cloud*"
QT_MOC_LITERAL(6, 57, 17), // "sendingVegetation"
QT_MOC_LITERAL(7, 75, 13), // "setResolution"
QT_MOC_LITERAL(8, 89, 3), // "res"
QT_MOC_LITERAL(9, 93, 12), // "setBaseCloud"
QT_MOC_LITERAL(10, 106, 5), // "Cloud"
QT_MOC_LITERAL(11, 112, 5), // "input"
QT_MOC_LITERAL(12, 118, 17), // "setVegetationName"
QT_MOC_LITERAL(13, 136, 4), // "name"
QT_MOC_LITERAL(14, 141, 14), // "setTerrainName"
QT_MOC_LITERAL(15, 156, 7), // "execute"
QT_MOC_LITERAL(16, 164, 6), // "octree"
QT_MOC_LITERAL(17, 171, 36), // "pcl::PointCloud<pcl::PointXYZ..."
QT_MOC_LITERAL(18, 208, 13), // "output_ground"
QT_MOC_LITERAL(19, 222, 11), // "output_vege"
QT_MOC_LITERAL(20, 234, 8), // "sendData"
QT_MOC_LITERAL(21, 243, 6) // "hotovo"

    },
    "OctreeTerrain\0finished\0\0percentage\0"
    "sendingTerrain\0Cloud*\0sendingVegetation\0"
    "setResolution\0res\0setBaseCloud\0Cloud\0"
    "input\0setVegetationName\0name\0"
    "setTerrainName\0execute\0octree\0"
    "pcl::PointCloud<pcl::PointXYZI>::Ptr\0"
    "output_ground\0output_vege\0sendData\0"
    "hotovo"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_OctreeTerrain[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x06 /* Public */,
       3,    1,   75,    2, 0x06 /* Public */,
       4,    1,   78,    2, 0x06 /* Public */,
       6,    1,   81,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    1,   84,    2, 0x0a /* Public */,
       9,    1,   87,    2, 0x0a /* Public */,
      12,    1,   90,    2, 0x0a /* Public */,
      14,    1,   93,    2, 0x0a /* Public */,
      15,    0,   96,    2, 0x0a /* Public */,
      16,    4,   97,    2, 0x0a /* Public */,
      20,    0,  106,    2, 0x0a /* Public */,
      21,    0,  107,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, 0x80000000 | 5,    2,
    QMetaType::Void, 0x80000000 | 5,    2,

 // slots: parameters
    QMetaType::Void, QMetaType::Float,    8,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, QMetaType::QString,   13,
    QMetaType::Void, QMetaType::QString,   13,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Float, 0x80000000 | 17, 0x80000000 | 17, 0x80000000 | 17,    8,   11,   18,   19,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void OctreeTerrain::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        OctreeTerrain *_t = static_cast<OctreeTerrain *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->finished(); break;
        case 1: _t->percentage((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->sendingTerrain((*reinterpret_cast< Cloud*(*)>(_a[1]))); break;
        case 3: _t->sendingVegetation((*reinterpret_cast< Cloud*(*)>(_a[1]))); break;
        case 4: _t->setResolution((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 5: _t->setBaseCloud((*reinterpret_cast< Cloud(*)>(_a[1]))); break;
        case 6: _t->setVegetationName((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->setTerrainName((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 8: _t->execute(); break;
        case 9: _t->octree((*reinterpret_cast< float(*)>(_a[1])),(*reinterpret_cast< pcl::PointCloud<pcl::PointXYZI>::Ptr(*)>(_a[2])),(*reinterpret_cast< pcl::PointCloud<pcl::PointXYZI>::Ptr(*)>(_a[3])),(*reinterpret_cast< pcl::PointCloud<pcl::PointXYZI>::Ptr(*)>(_a[4]))); break;
        case 10: _t->sendData(); break;
        case 11: _t->hotovo(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (OctreeTerrain::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&OctreeTerrain::finished)) {
                *result = 0;
            }
        }
        {
            typedef void (OctreeTerrain::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&OctreeTerrain::percentage)) {
                *result = 1;
            }
        }
        {
            typedef void (OctreeTerrain::*_t)(Cloud * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&OctreeTerrain::sendingTerrain)) {
                *result = 2;
            }
        }
        {
            typedef void (OctreeTerrain::*_t)(Cloud * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&OctreeTerrain::sendingVegetation)) {
                *result = 3;
            }
        }
    }
}

const QMetaObject OctreeTerrain::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_OctreeTerrain.data,
      qt_meta_data_OctreeTerrain,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *OctreeTerrain::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *OctreeTerrain::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_OctreeTerrain.stringdata0))
        return static_cast<void*>(const_cast< OctreeTerrain*>(this));
    return QObject::qt_metacast(_clname);
}

int OctreeTerrain::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}

// SIGNAL 0
void OctreeTerrain::finished()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void OctreeTerrain::percentage(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void OctreeTerrain::sendingTerrain(Cloud * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void OctreeTerrain::sendingVegetation(Cloud * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
struct qt_meta_stringdata_VoxelTerrain_t {
    QByteArrayData data[18];
    char stringdata0[179];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_VoxelTerrain_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_VoxelTerrain_t qt_meta_stringdata_VoxelTerrain = {
    {
QT_MOC_LITERAL(0, 0, 12), // "VoxelTerrain"
QT_MOC_LITERAL(1, 13, 8), // "finished"
QT_MOC_LITERAL(2, 22, 0), // ""
QT_MOC_LITERAL(3, 23, 10), // "percentage"
QT_MOC_LITERAL(4, 34, 14), // "sendingTerrain"
QT_MOC_LITERAL(5, 49, 6), // "Cloud*"
QT_MOC_LITERAL(6, 56, 17), // "sendingVegetation"
QT_MOC_LITERAL(7, 74, 13), // "setResolution"
QT_MOC_LITERAL(8, 88, 3), // "res"
QT_MOC_LITERAL(9, 92, 12), // "setBaseCloud"
QT_MOC_LITERAL(10, 105, 5), // "Cloud"
QT_MOC_LITERAL(11, 111, 5), // "input"
QT_MOC_LITERAL(12, 117, 17), // "setVegetationName"
QT_MOC_LITERAL(13, 135, 4), // "name"
QT_MOC_LITERAL(14, 140, 14), // "setTerrainName"
QT_MOC_LITERAL(15, 155, 7), // "execute"
QT_MOC_LITERAL(16, 163, 8), // "sendData"
QT_MOC_LITERAL(17, 172, 6) // "hotovo"

    },
    "VoxelTerrain\0finished\0\0percentage\0"
    "sendingTerrain\0Cloud*\0sendingVegetation\0"
    "setResolution\0res\0setBaseCloud\0Cloud\0"
    "input\0setVegetationName\0name\0"
    "setTerrainName\0execute\0sendData\0hotovo"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_VoxelTerrain[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      11,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   69,    2, 0x06 /* Public */,
       3,    1,   70,    2, 0x06 /* Public */,
       4,    1,   73,    2, 0x06 /* Public */,
       6,    1,   76,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       7,    1,   79,    2, 0x0a /* Public */,
       9,    1,   82,    2, 0x0a /* Public */,
      12,    1,   85,    2, 0x0a /* Public */,
      14,    1,   88,    2, 0x0a /* Public */,
      15,    0,   91,    2, 0x0a /* Public */,
      16,    0,   92,    2, 0x0a /* Public */,
      17,    0,   93,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, 0x80000000 | 5,    2,
    QMetaType::Void, 0x80000000 | 5,    2,

 // slots: parameters
    QMetaType::Void, QMetaType::Float,    8,
    QMetaType::Void, 0x80000000 | 10,   11,
    QMetaType::Void, QMetaType::QString,   13,
    QMetaType::Void, QMetaType::QString,   13,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void VoxelTerrain::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        VoxelTerrain *_t = static_cast<VoxelTerrain *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->finished(); break;
        case 1: _t->percentage((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->sendingTerrain((*reinterpret_cast< Cloud*(*)>(_a[1]))); break;
        case 3: _t->sendingVegetation((*reinterpret_cast< Cloud*(*)>(_a[1]))); break;
        case 4: _t->setResolution((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 5: _t->setBaseCloud((*reinterpret_cast< Cloud(*)>(_a[1]))); break;
        case 6: _t->setVegetationName((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->setTerrainName((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 8: _t->execute(); break;
        case 9: _t->sendData(); break;
        case 10: _t->hotovo(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (VoxelTerrain::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&VoxelTerrain::finished)) {
                *result = 0;
            }
        }
        {
            typedef void (VoxelTerrain::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&VoxelTerrain::percentage)) {
                *result = 1;
            }
        }
        {
            typedef void (VoxelTerrain::*_t)(Cloud * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&VoxelTerrain::sendingTerrain)) {
                *result = 2;
            }
        }
        {
            typedef void (VoxelTerrain::*_t)(Cloud * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&VoxelTerrain::sendingVegetation)) {
                *result = 3;
            }
        }
    }
}

const QMetaObject VoxelTerrain::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_VoxelTerrain.data,
      qt_meta_data_VoxelTerrain,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *VoxelTerrain::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *VoxelTerrain::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_VoxelTerrain.stringdata0))
        return static_cast<void*>(const_cast< VoxelTerrain*>(this));
    return QObject::qt_metacast(_clname);
}

int VoxelTerrain::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 11)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 11;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 11)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 11;
    }
    return _id;
}

// SIGNAL 0
void VoxelTerrain::finished()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void VoxelTerrain::percentage(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void VoxelTerrain::sendingTerrain(Cloud * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void VoxelTerrain::sendingVegetation(Cloud * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
struct qt_meta_stringdata_IDW_t {
    QByteArrayData data[18];
    char stringdata0[151];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_IDW_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_IDW_t qt_meta_stringdata_IDW = {
    {
QT_MOC_LITERAL(0, 0, 3), // "IDW"
QT_MOC_LITERAL(1, 4, 8), // "finished"
QT_MOC_LITERAL(2, 13, 0), // ""
QT_MOC_LITERAL(3, 14, 10), // "percentage"
QT_MOC_LITERAL(4, 25, 13), // "sendingoutput"
QT_MOC_LITERAL(5, 39, 6), // "Cloud*"
QT_MOC_LITERAL(6, 46, 13), // "setResolution"
QT_MOC_LITERAL(7, 60, 3), // "res"
QT_MOC_LITERAL(8, 64, 14), // "setPointNumber"
QT_MOC_LITERAL(9, 79, 3), // "num"
QT_MOC_LITERAL(10, 83, 12), // "setBaseCloud"
QT_MOC_LITERAL(11, 96, 5), // "Cloud"
QT_MOC_LITERAL(12, 102, 5), // "input"
QT_MOC_LITERAL(13, 108, 13), // "setOutputName"
QT_MOC_LITERAL(14, 122, 4), // "name"
QT_MOC_LITERAL(15, 127, 7), // "execute"
QT_MOC_LITERAL(16, 135, 8), // "sendData"
QT_MOC_LITERAL(17, 144, 6) // "hotovo"

    },
    "IDW\0finished\0\0percentage\0sendingoutput\0"
    "Cloud*\0setResolution\0res\0setPointNumber\0"
    "num\0setBaseCloud\0Cloud\0input\0setOutputName\0"
    "name\0execute\0sendData\0hotovo"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_IDW[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   64,    2, 0x06 /* Public */,
       3,    1,   65,    2, 0x06 /* Public */,
       4,    1,   68,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    1,   71,    2, 0x0a /* Public */,
       8,    1,   74,    2, 0x0a /* Public */,
      10,    1,   77,    2, 0x0a /* Public */,
      13,    1,   80,    2, 0x0a /* Public */,
      15,    0,   83,    2, 0x0a /* Public */,
      16,    0,   84,    2, 0x0a /* Public */,
      17,    0,   85,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, 0x80000000 | 5,    2,

 // slots: parameters
    QMetaType::Void, QMetaType::Float,    7,
    QMetaType::Void, QMetaType::Float,    9,
    QMetaType::Void, 0x80000000 | 11,   12,
    QMetaType::Void, QMetaType::QString,   14,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void IDW::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        IDW *_t = static_cast<IDW *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->finished(); break;
        case 1: _t->percentage((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->sendingoutput((*reinterpret_cast< Cloud*(*)>(_a[1]))); break;
        case 3: _t->setResolution((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 4: _t->setPointNumber((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 5: _t->setBaseCloud((*reinterpret_cast< Cloud(*)>(_a[1]))); break;
        case 6: _t->setOutputName((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->execute(); break;
        case 8: _t->sendData(); break;
        case 9: _t->hotovo(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (IDW::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&IDW::finished)) {
                *result = 0;
            }
        }
        {
            typedef void (IDW::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&IDW::percentage)) {
                *result = 1;
            }
        }
        {
            typedef void (IDW::*_t)(Cloud * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&IDW::sendingoutput)) {
                *result = 2;
            }
        }
    }
}

const QMetaObject IDW::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_IDW.data,
      qt_meta_data_IDW,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *IDW::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *IDW::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_IDW.stringdata0))
        return static_cast<void*>(const_cast< IDW*>(this));
    return QObject::qt_metacast(_clname);
}

int IDW::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void IDW::finished()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void IDW::percentage(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void IDW::sendingoutput(Cloud * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
struct qt_meta_stringdata_StatOutlierRemoval_t {
    QByteArrayData data[18];
    char stringdata0[170];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_StatOutlierRemoval_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_StatOutlierRemoval_t qt_meta_stringdata_StatOutlierRemoval = {
    {
QT_MOC_LITERAL(0, 0, 18), // "StatOutlierRemoval"
QT_MOC_LITERAL(1, 19, 8), // "finished"
QT_MOC_LITERAL(2, 28, 0), // ""
QT_MOC_LITERAL(3, 29, 10), // "percentage"
QT_MOC_LITERAL(4, 40, 13), // "sendingoutput"
QT_MOC_LITERAL(5, 54, 6), // "Cloud*"
QT_MOC_LITERAL(6, 61, 15), // "setMeanDistance"
QT_MOC_LITERAL(7, 77, 4), // "dist"
QT_MOC_LITERAL(8, 82, 15), // "setNeighborhood"
QT_MOC_LITERAL(9, 98, 3), // "num"
QT_MOC_LITERAL(10, 102, 12), // "setBaseCloud"
QT_MOC_LITERAL(11, 115, 5), // "Cloud"
QT_MOC_LITERAL(12, 121, 5), // "input"
QT_MOC_LITERAL(13, 127, 13), // "setOutputName"
QT_MOC_LITERAL(14, 141, 4), // "name"
QT_MOC_LITERAL(15, 146, 7), // "execute"
QT_MOC_LITERAL(16, 154, 8), // "sendData"
QT_MOC_LITERAL(17, 163, 6) // "hotovo"

    },
    "StatOutlierRemoval\0finished\0\0percentage\0"
    "sendingoutput\0Cloud*\0setMeanDistance\0"
    "dist\0setNeighborhood\0num\0setBaseCloud\0"
    "Cloud\0input\0setOutputName\0name\0execute\0"
    "sendData\0hotovo"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_StatOutlierRemoval[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   64,    2, 0x06 /* Public */,
       3,    1,   65,    2, 0x06 /* Public */,
       4,    1,   68,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    1,   71,    2, 0x0a /* Public */,
       8,    1,   74,    2, 0x0a /* Public */,
      10,    1,   77,    2, 0x0a /* Public */,
      13,    1,   80,    2, 0x0a /* Public */,
      15,    0,   83,    2, 0x0a /* Public */,
      16,    0,   84,    2, 0x0a /* Public */,
      17,    0,   85,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, 0x80000000 | 5,    2,

 // slots: parameters
    QMetaType::Void, QMetaType::Float,    7,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, 0x80000000 | 11,   12,
    QMetaType::Void, QMetaType::QString,   14,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void StatOutlierRemoval::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        StatOutlierRemoval *_t = static_cast<StatOutlierRemoval *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->finished(); break;
        case 1: _t->percentage((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->sendingoutput((*reinterpret_cast< Cloud*(*)>(_a[1]))); break;
        case 3: _t->setMeanDistance((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 4: _t->setNeighborhood((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->setBaseCloud((*reinterpret_cast< Cloud(*)>(_a[1]))); break;
        case 6: _t->setOutputName((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->execute(); break;
        case 8: _t->sendData(); break;
        case 9: _t->hotovo(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (StatOutlierRemoval::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&StatOutlierRemoval::finished)) {
                *result = 0;
            }
        }
        {
            typedef void (StatOutlierRemoval::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&StatOutlierRemoval::percentage)) {
                *result = 1;
            }
        }
        {
            typedef void (StatOutlierRemoval::*_t)(Cloud * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&StatOutlierRemoval::sendingoutput)) {
                *result = 2;
            }
        }
    }
}

const QMetaObject StatOutlierRemoval::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_StatOutlierRemoval.data,
      qt_meta_data_StatOutlierRemoval,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *StatOutlierRemoval::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *StatOutlierRemoval::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_StatOutlierRemoval.stringdata0))
        return static_cast<void*>(const_cast< StatOutlierRemoval*>(this));
    return QObject::qt_metacast(_clname);
}

int StatOutlierRemoval::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void StatOutlierRemoval::finished()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void StatOutlierRemoval::percentage(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void StatOutlierRemoval::sendingoutput(Cloud * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
struct qt_meta_stringdata_RadiusOutlierRemoval_t {
    QByteArrayData data[18];
    char stringdata0[168];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_RadiusOutlierRemoval_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_RadiusOutlierRemoval_t qt_meta_stringdata_RadiusOutlierRemoval = {
    {
QT_MOC_LITERAL(0, 0, 20), // "RadiusOutlierRemoval"
QT_MOC_LITERAL(1, 21, 8), // "finished"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 10), // "percentage"
QT_MOC_LITERAL(4, 42, 13), // "sendingoutput"
QT_MOC_LITERAL(5, 56, 6), // "Cloud*"
QT_MOC_LITERAL(6, 63, 9), // "setRadius"
QT_MOC_LITERAL(7, 73, 6), // "radius"
QT_MOC_LITERAL(8, 80, 15), // "setNeighborhood"
QT_MOC_LITERAL(9, 96, 3), // "num"
QT_MOC_LITERAL(10, 100, 12), // "setBaseCloud"
QT_MOC_LITERAL(11, 113, 5), // "Cloud"
QT_MOC_LITERAL(12, 119, 5), // "input"
QT_MOC_LITERAL(13, 125, 13), // "setOutputName"
QT_MOC_LITERAL(14, 139, 4), // "name"
QT_MOC_LITERAL(15, 144, 7), // "execute"
QT_MOC_LITERAL(16, 152, 8), // "sendData"
QT_MOC_LITERAL(17, 161, 6) // "hotovo"

    },
    "RadiusOutlierRemoval\0finished\0\0"
    "percentage\0sendingoutput\0Cloud*\0"
    "setRadius\0radius\0setNeighborhood\0num\0"
    "setBaseCloud\0Cloud\0input\0setOutputName\0"
    "name\0execute\0sendData\0hotovo"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_RadiusOutlierRemoval[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   64,    2, 0x06 /* Public */,
       3,    1,   65,    2, 0x06 /* Public */,
       4,    1,   68,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    1,   71,    2, 0x0a /* Public */,
       8,    1,   74,    2, 0x0a /* Public */,
      10,    1,   77,    2, 0x0a /* Public */,
      13,    1,   80,    2, 0x0a /* Public */,
      15,    0,   83,    2, 0x0a /* Public */,
      16,    0,   84,    2, 0x0a /* Public */,
      17,    0,   85,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, 0x80000000 | 5,    2,

 // slots: parameters
    QMetaType::Void, QMetaType::Float,    7,
    QMetaType::Void, QMetaType::Int,    9,
    QMetaType::Void, 0x80000000 | 11,   12,
    QMetaType::Void, QMetaType::QString,   14,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void RadiusOutlierRemoval::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        RadiusOutlierRemoval *_t = static_cast<RadiusOutlierRemoval *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->finished(); break;
        case 1: _t->percentage((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->sendingoutput((*reinterpret_cast< Cloud*(*)>(_a[1]))); break;
        case 3: _t->setRadius((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 4: _t->setNeighborhood((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->setBaseCloud((*reinterpret_cast< Cloud(*)>(_a[1]))); break;
        case 6: _t->setOutputName((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->execute(); break;
        case 8: _t->sendData(); break;
        case 9: _t->hotovo(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (RadiusOutlierRemoval::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&RadiusOutlierRemoval::finished)) {
                *result = 0;
            }
        }
        {
            typedef void (RadiusOutlierRemoval::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&RadiusOutlierRemoval::percentage)) {
                *result = 1;
            }
        }
        {
            typedef void (RadiusOutlierRemoval::*_t)(Cloud * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&RadiusOutlierRemoval::sendingoutput)) {
                *result = 2;
            }
        }
    }
}

const QMetaObject RadiusOutlierRemoval::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_RadiusOutlierRemoval.data,
      qt_meta_data_RadiusOutlierRemoval,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *RadiusOutlierRemoval::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *RadiusOutlierRemoval::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_RadiusOutlierRemoval.stringdata0))
        return static_cast<void*>(const_cast< RadiusOutlierRemoval*>(this));
    return QObject::qt_metacast(_clname);
}

int RadiusOutlierRemoval::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 10)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void RadiusOutlierRemoval::finished()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void RadiusOutlierRemoval::percentage(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void RadiusOutlierRemoval::sendingoutput(Cloud * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}
QT_END_MOC_NAMESPACE
