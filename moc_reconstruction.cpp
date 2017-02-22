/****************************************************************************
** Meta object code from reading C++ file 'reconstruction.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "reconstruction.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'reconstruction.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_TreeReconstruction_t {
    QByteArrayData data[27];
    char stringdata0[327];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_TreeReconstruction_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_TreeReconstruction_t qt_meta_stringdata_TreeReconstruction = {
    {
QT_MOC_LITERAL(0, 0, 18), // "TreeReconstruction"
QT_MOC_LITERAL(1, 19, 7), // "started"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 8), // "finished"
QT_MOC_LITERAL(4, 37, 11), // "sendingTree"
QT_MOC_LITERAL(5, 49, 6), // "Cloud*"
QT_MOC_LITERAL(6, 56, 12), // "sendingCentr"
QT_MOC_LITERAL(7, 69, 6), // "hotovo"
QT_MOC_LITERAL(8, 76, 10), // "percentage"
QT_MOC_LITERAL(9, 87, 11), // "setDistance"
QT_MOC_LITERAL(10, 99, 1), // "i"
QT_MOC_LITERAL(11, 101, 15), // "setMinimalPoint"
QT_MOC_LITERAL(12, 117, 7), // "setTree"
QT_MOC_LITERAL(13, 125, 36), // "pcl::PointCloud<pcl::PointXYZ..."
QT_MOC_LITERAL(14, 162, 4), // "tree"
QT_MOC_LITERAL(15, 167, 16), // "setCentroidsName"
QT_MOC_LITERAL(16, 184, 4), // "name"
QT_MOC_LITERAL(17, 189, 16), // "euclSegmentation"
QT_MOC_LITERAL(18, 206, 5), // "input"
QT_MOC_LITERAL(19, 212, 50), // "std::vector<pcl::PointCloud<p..."
QT_MOC_LITERAL(20, 263, 8), // "clusters"
QT_MOC_LITERAL(21, 272, 6), // "min_pt"
QT_MOC_LITERAL(22, 279, 7), // "cluster"
QT_MOC_LITERAL(23, 287, 12), // "stemSkeleton"
QT_MOC_LITERAL(24, 300, 9), // "setVertex"
QT_MOC_LITERAL(25, 310, 7), // "execute"
QT_MOC_LITERAL(26, 318, 8) // "sendData"

    },
    "TreeReconstruction\0started\0\0finished\0"
    "sendingTree\0Cloud*\0sendingCentr\0hotovo\0"
    "percentage\0setDistance\0i\0setMinimalPoint\0"
    "setTree\0pcl::PointCloud<pcl::PointXYZI>::Ptr\0"
    "tree\0setCentroidsName\0name\0euclSegmentation\0"
    "input\0std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>&\0"
    "clusters\0min_pt\0cluster\0stemSkeleton\0"
    "setVertex\0execute\0sendData"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_TreeReconstruction[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   94,    2, 0x06 /* Public */,
       3,    0,   95,    2, 0x06 /* Public */,
       4,    1,   96,    2, 0x06 /* Public */,
       6,    1,   99,    2, 0x06 /* Public */,
       7,    0,  102,    2, 0x06 /* Public */,
       8,    1,  103,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       9,    1,  106,    2, 0x0a /* Public */,
      11,    1,  109,    2, 0x0a /* Public */,
      12,    1,  112,    2, 0x0a /* Public */,
      15,    1,  115,    2, 0x0a /* Public */,
      17,    3,  118,    2, 0x0a /* Public */,
      22,    0,  125,    2, 0x0a /* Public */,
      23,    0,  126,    2, 0x0a /* Public */,
      24,    0,  127,    2, 0x0a /* Public */,
      25,    0,  128,    2, 0x0a /* Public */,
      26,    0,  129,    2, 0x0a /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5,    2,
    QMetaType::Void, 0x80000000 | 5,    2,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    2,

 // slots: parameters
    QMetaType::Void, QMetaType::Float,   10,
    QMetaType::Void, QMetaType::Int,   10,
    QMetaType::Void, 0x80000000 | 13,   14,
    QMetaType::Void, QMetaType::QString,   16,
    QMetaType::Int, 0x80000000 | 13, 0x80000000 | 19, QMetaType::Int,   18,   20,   21,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void TreeReconstruction::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        TreeReconstruction *_t = static_cast<TreeReconstruction *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->started(); break;
        case 1: _t->finished(); break;
        case 2: _t->sendingTree((*reinterpret_cast< Cloud*(*)>(_a[1]))); break;
        case 3: _t->sendingCentr((*reinterpret_cast< Cloud*(*)>(_a[1]))); break;
        case 4: _t->hotovo(); break;
        case 5: _t->percentage((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->setDistance((*reinterpret_cast< float(*)>(_a[1]))); break;
        case 7: _t->setMinimalPoint((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->setTree((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZI>::Ptr(*)>(_a[1]))); break;
        case 9: _t->setCentroidsName((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 10: { int _r = _t->euclSegmentation((*reinterpret_cast< pcl::PointCloud<pcl::PointXYZI>::Ptr(*)>(_a[1])),(*reinterpret_cast< std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>(*)>(_a[2])),(*reinterpret_cast< int(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 11: _t->cluster(); break;
        case 12: _t->stemSkeleton(); break;
        case 13: _t->setVertex(); break;
        case 14: _t->execute(); break;
        case 15: _t->sendData(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (TreeReconstruction::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TreeReconstruction::started)) {
                *result = 0;
            }
        }
        {
            typedef void (TreeReconstruction::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TreeReconstruction::finished)) {
                *result = 1;
            }
        }
        {
            typedef void (TreeReconstruction::*_t)(Cloud * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TreeReconstruction::sendingTree)) {
                *result = 2;
            }
        }
        {
            typedef void (TreeReconstruction::*_t)(Cloud * );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TreeReconstruction::sendingCentr)) {
                *result = 3;
            }
        }
        {
            typedef void (TreeReconstruction::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TreeReconstruction::hotovo)) {
                *result = 4;
            }
        }
        {
            typedef void (TreeReconstruction::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&TreeReconstruction::percentage)) {
                *result = 5;
            }
        }
    }
}

const QMetaObject TreeReconstruction::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_TreeReconstruction.data,
      qt_meta_data_TreeReconstruction,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *TreeReconstruction::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *TreeReconstruction::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_TreeReconstruction.stringdata0))
        return static_cast<void*>(const_cast< TreeReconstruction*>(this));
    return QObject::qt_metacast(_clname);
}

int TreeReconstruction::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 16)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 16;
    }
    return _id;
}

// SIGNAL 0
void TreeReconstruction::started()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void TreeReconstruction::finished()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void TreeReconstruction::sendingTree(Cloud * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void TreeReconstruction::sendingCentr(Cloud * _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void TreeReconstruction::hotovo()
{
    QMetaObject::activate(this, &staticMetaObject, 4, Q_NULLPTR);
}

// SIGNAL 5
void TreeReconstruction::percentage(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_END_MOC_NAMESPACE
