/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[161];
    char stringdata0[2429];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 9), // "savedVege"
QT_MOC_LITERAL(2, 21, 0), // ""
QT_MOC_LITERAL(3, 22, 12), // "savedTerrain"
QT_MOC_LITERAL(4, 35, 9), // "savedTree"
QT_MOC_LITERAL(5, 45, 9), // "savedRest"
QT_MOC_LITERAL(6, 55, 8), // "get_path"
QT_MOC_LITERAL(7, 64, 10), // "newProject"
QT_MOC_LITERAL(8, 75, 11), // "openProject"
QT_MOC_LITERAL(9, 87, 12), // "closeProject"
QT_MOC_LITERAL(10, 100, 13), // "importProject"
QT_MOC_LITERAL(11, 114, 18), // "showAttributeTable"
QT_MOC_LITERAL(12, 133, 17), // "importTerrainFile"
QT_MOC_LITERAL(13, 151, 15), // "importBaseCloud"
QT_MOC_LITERAL(14, 167, 15), // "importVegeCloud"
QT_MOC_LITERAL(15, 183, 15), // "importTreeCloud"
QT_MOC_LITERAL(16, 199, 14), // "importOstCloud"
QT_MOC_LITERAL(17, 214, 11), // "exportCloud"
QT_MOC_LITERAL(18, 226, 15), // "exportConvexTxt"
QT_MOC_LITERAL(19, 242, 16), // "exportConcaveTxt"
QT_MOC_LITERAL(20, 259, 10), // "closeEvent"
QT_MOC_LITERAL(21, 270, 12), // "QCloseEvent*"
QT_MOC_LITERAL(22, 283, 5), // "event"
QT_MOC_LITERAL(23, 289, 9), // "voxelgrid"
QT_MOC_LITERAL(24, 299, 10), // "octreeSlot"
QT_MOC_LITERAL(25, 310, 12), // "manualAdjust"
QT_MOC_LITERAL(26, 323, 16), // "manualAdjustStop"
QT_MOC_LITERAL(27, 340, 7), // "IDWslot"
QT_MOC_LITERAL(28, 348, 25), // "statisticalOutlierRemoval"
QT_MOC_LITERAL(29, 374, 20), // "radiusOutlierRemoval"
QT_MOC_LITERAL(30, 395, 12), // "manualSelect"
QT_MOC_LITERAL(31, 408, 16), // "manualSelectStop"
QT_MOC_LITERAL(32, 425, 12), // "segmentation"
QT_MOC_LITERAL(33, 438, 16), // "manualSelectExit"
QT_MOC_LITERAL(34, 455, 8), // "treeEdit"
QT_MOC_LITERAL(35, 464, 12), // "treeEditStop"
QT_MOC_LITERAL(36, 477, 13), // "treeAtributes"
QT_MOC_LITERAL(37, 491, 5), // "dbhHT"
QT_MOC_LITERAL(38, 497, 12), // "dbhHTDisplay"
QT_MOC_LITERAL(39, 510, 4), // "name"
QT_MOC_LITERAL(40, 515, 16), // "dbhHT_DisplayAll"
QT_MOC_LITERAL(41, 532, 13), // "dbhHT_HideAll"
QT_MOC_LITERAL(42, 546, 6), // "dbhLSR"
QT_MOC_LITERAL(43, 553, 8), // "dbhCheck"
QT_MOC_LITERAL(44, 562, 13), // "dbhLSRDisplay"
QT_MOC_LITERAL(45, 576, 17), // "dbhLSR_DisplayAll"
QT_MOC_LITERAL(46, 594, 14), // "dbhLSR_HideAll"
QT_MOC_LITERAL(47, 609, 6), // "height"
QT_MOC_LITERAL(48, 616, 13), // "heightDisplay"
QT_MOC_LITERAL(49, 630, 17), // "height_DisplayAll"
QT_MOC_LITERAL(50, 648, 14), // "height_HideAll"
QT_MOC_LITERAL(51, 663, 8), // "position"
QT_MOC_LITERAL(52, 672, 10), // "positionHT"
QT_MOC_LITERAL(53, 683, 15), // "positionDisplay"
QT_MOC_LITERAL(54, 699, 19), // "position_DisplayAll"
QT_MOC_LITERAL(55, 719, 16), // "position_HideAll"
QT_MOC_LITERAL(56, 736, 6), // "length"
QT_MOC_LITERAL(57, 743, 13), // "lengthDisplay"
QT_MOC_LITERAL(58, 757, 17), // "length_DisplayAll"
QT_MOC_LITERAL(59, 775, 14), // "length_HideAll"
QT_MOC_LITERAL(60, 790, 8), // "skeleton"
QT_MOC_LITERAL(61, 799, 15), // "skeletonDisplay"
QT_MOC_LITERAL(62, 815, 19), // "skeleton_DisplayAll"
QT_MOC_LITERAL(63, 835, 16), // "skeleton_HideAll"
QT_MOC_LITERAL(64, 852, 10), // "convexhull"
QT_MOC_LITERAL(65, 863, 17), // "convexhullDisplay"
QT_MOC_LITERAL(66, 881, 21), // "convexhull_DisplayAll"
QT_MOC_LITERAL(67, 903, 18), // "convexhull_HideAll"
QT_MOC_LITERAL(68, 922, 11), // "concavehull"
QT_MOC_LITERAL(69, 934, 18), // "concavehullDisplay"
QT_MOC_LITERAL(70, 953, 22), // "concavehull_DisplayAll"
QT_MOC_LITERAL(71, 976, 19), // "concavehull_HideAll"
QT_MOC_LITERAL(72, 996, 13), // "stemCurvature"
QT_MOC_LITERAL(73, 1010, 20), // "stemCurvatureDisplay"
QT_MOC_LITERAL(74, 1031, 24), // "stemCurvature_DisplayAll"
QT_MOC_LITERAL(75, 1056, 21), // "stemCurvature_HideAll"
QT_MOC_LITERAL(76, 1078, 19), // "stemCurvatureExport"
QT_MOC_LITERAL(77, 1098, 12), // "dbhCloudEdit"
QT_MOC_LITERAL(78, 1111, 16), // "dbhCloudStopEdit"
QT_MOC_LITERAL(79, 1128, 14), // "reconstruction"
QT_MOC_LITERAL(80, 1143, 15), // "set_CrownManual"
QT_MOC_LITERAL(81, 1159, 15), // "CrownManualStop"
QT_MOC_LITERAL(82, 1175, 18), // "set_CrownAutomatic"
QT_MOC_LITERAL(83, 1194, 12), // "crownDisplay"
QT_MOC_LITERAL(84, 1207, 16), // "crown_DisplayAll"
QT_MOC_LITERAL(85, 1224, 13), // "crown_HideAll"
QT_MOC_LITERAL(86, 1238, 18), // "CrownHeightDisplay"
QT_MOC_LITERAL(87, 1257, 22), // "crownHeightsDisplayAll"
QT_MOC_LITERAL(88, 1280, 19), // "crownHeightsHideAll"
QT_MOC_LITERAL(89, 1300, 20), // "crownPositionDisplay"
QT_MOC_LITERAL(90, 1321, 23), // "crownPositionDisplayAll"
QT_MOC_LITERAL(91, 1345, 20), // "crownPositionHideAll"
QT_MOC_LITERAL(92, 1366, 32), // "setSectionsVolumeSurfacePosition"
QT_MOC_LITERAL(93, 1399, 29), // "crownSurfaceBySectionsHideAll"
QT_MOC_LITERAL(94, 1429, 32), // "crownSurfaceBySectionsDisplayAll"
QT_MOC_LITERAL(95, 1462, 33), // "crownSurfaceBySectionsDisplay..."
QT_MOC_LITERAL(96, 1496, 17), // "create3DConvexull"
QT_MOC_LITERAL(97, 1514, 28), // "crownSurface3DHullDisplayAll"
QT_MOC_LITERAL(98, 1543, 25), // "crownSurface3DHullHideAll"
QT_MOC_LITERAL(99, 1569, 29), // "crownSurface3DHullDisplayName"
QT_MOC_LITERAL(100, 1599, 26), // "crownExternalPtsDisplayAll"
QT_MOC_LITERAL(101, 1626, 23), // "crownExternalPtsHideAll"
QT_MOC_LITERAL(102, 1650, 26), // "computeCrownsIntersections"
QT_MOC_LITERAL(103, 1677, 20), // "intersectionsShowAll"
QT_MOC_LITERAL(104, 1698, 20), // "intersectionsHideAll"
QT_MOC_LITERAL(105, 1719, 27), // "showCrownIntersectionsTable"
QT_MOC_LITERAL(106, 1747, 21), // "exportCrownAttributes"
QT_MOC_LITERAL(107, 1769, 19), // "exportIntersections"
QT_MOC_LITERAL(108, 1789, 27), // "recomputeAfterTreePosChenge"
QT_MOC_LITERAL(109, 1817, 19), // "crownVolumeByVoxels"
QT_MOC_LITERAL(110, 1837, 19), // "eraseSelectedClouds"
QT_MOC_LITERAL(111, 1857, 15), // "mergeCloudsByID"
QT_MOC_LITERAL(112, 1873, 11), // "labelClouds"
QT_MOC_LITERAL(113, 1885, 14), // "labelCloudsOFF"
QT_MOC_LITERAL(114, 1900, 11), // "mergeClouds"
QT_MOC_LITERAL(115, 1912, 10), // "minusCloud"
QT_MOC_LITERAL(116, 1923, 10), // "splitCloud"
QT_MOC_LITERAL(117, 1934, 8), // "voxelize"
QT_MOC_LITERAL(118, 1943, 8), // "accuracy"
QT_MOC_LITERAL(119, 1952, 15), // "duplicatePoints"
QT_MOC_LITERAL(120, 1968, 7), // "topView"
QT_MOC_LITERAL(121, 1976, 10), // "bottomView"
QT_MOC_LITERAL(122, 1987, 9), // "frontView"
QT_MOC_LITERAL(123, 1997, 8), // "backView"
QT_MOC_LITERAL(124, 2006, 9), // "sideAView"
QT_MOC_LITERAL(125, 2016, 9), // "sideBView"
QT_MOC_LITERAL(126, 2026, 11), // "perspective"
QT_MOC_LITERAL(127, 2038, 5), // "ortho"
QT_MOC_LITERAL(128, 2044, 16), // "set_ConcaveCloud"
QT_MOC_LITERAL(129, 2061, 15), // "set_ConvexCloud"
QT_MOC_LITERAL(130, 2077, 9), // "save_tiff"
QT_MOC_LITERAL(131, 2087, 7), // "bgColor"
QT_MOC_LITERAL(132, 2095, 5), // "about"
QT_MOC_LITERAL(133, 2101, 9), // "dispCloud"
QT_MOC_LITERAL(134, 2111, 11), // "removeCloud"
QT_MOC_LITERAL(135, 2123, 11), // "deleteCloud"
QT_MOC_LITERAL(136, 2135, 10), // "colorCloud"
QT_MOC_LITERAL(137, 2146, 15), // "colorCloudField"
QT_MOC_LITERAL(138, 2162, 9), // "PointSize"
QT_MOC_LITERAL(139, 2172, 4), // "undo"
QT_MOC_LITERAL(140, 2177, 20), // "displayHideEditCloud"
QT_MOC_LITERAL(141, 2198, 14), // "saveVegetation"
QT_MOC_LITERAL(142, 2213, 6), // "Cloud*"
QT_MOC_LITERAL(143, 2220, 11), // "saveTerrain"
QT_MOC_LITERAL(144, 2232, 8), // "saveTree"
QT_MOC_LITERAL(145, 2241, 8), // "saveRest"
QT_MOC_LITERAL(146, 2250, 1), // "c"
QT_MOC_LITERAL(147, 2252, 25), // "showProgressBar100percent"
QT_MOC_LITERAL(148, 2278, 17), // "showProgressBarAt"
QT_MOC_LITERAL(149, 2296, 13), // "QProgressBar*"
QT_MOC_LITERAL(150, 2310, 4), // "pBar"
QT_MOC_LITERAL(151, 2315, 1), // "a"
QT_MOC_LITERAL(152, 2317, 23), // "showProgressBarInfinity"
QT_MOC_LITERAL(153, 2341, 10), // "createPBar"
QT_MOC_LITERAL(154, 2352, 13), // "showPBarValue"
QT_MOC_LITERAL(155, 2366, 10), // "removePbar"
QT_MOC_LITERAL(156, 2377, 9), // "nextSlice"
QT_MOC_LITERAL(157, 2387, 9), // "prevSlice"
QT_MOC_LITERAL(158, 2397, 5), // "slice"
QT_MOC_LITERAL(159, 2403, 9), // "sliceStop"
QT_MOC_LITERAL(160, 2413, 15) // "refreshAttTable"

    },
    "MainWindow\0savedVege\0\0savedTerrain\0"
    "savedTree\0savedRest\0get_path\0newProject\0"
    "openProject\0closeProject\0importProject\0"
    "showAttributeTable\0importTerrainFile\0"
    "importBaseCloud\0importVegeCloud\0"
    "importTreeCloud\0importOstCloud\0"
    "exportCloud\0exportConvexTxt\0"
    "exportConcaveTxt\0closeEvent\0QCloseEvent*\0"
    "event\0voxelgrid\0octreeSlot\0manualAdjust\0"
    "manualAdjustStop\0IDWslot\0"
    "statisticalOutlierRemoval\0"
    "radiusOutlierRemoval\0manualSelect\0"
    "manualSelectStop\0segmentation\0"
    "manualSelectExit\0treeEdit\0treeEditStop\0"
    "treeAtributes\0dbhHT\0dbhHTDisplay\0name\0"
    "dbhHT_DisplayAll\0dbhHT_HideAll\0dbhLSR\0"
    "dbhCheck\0dbhLSRDisplay\0dbhLSR_DisplayAll\0"
    "dbhLSR_HideAll\0height\0heightDisplay\0"
    "height_DisplayAll\0height_HideAll\0"
    "position\0positionHT\0positionDisplay\0"
    "position_DisplayAll\0position_HideAll\0"
    "length\0lengthDisplay\0length_DisplayAll\0"
    "length_HideAll\0skeleton\0skeletonDisplay\0"
    "skeleton_DisplayAll\0skeleton_HideAll\0"
    "convexhull\0convexhullDisplay\0"
    "convexhull_DisplayAll\0convexhull_HideAll\0"
    "concavehull\0concavehullDisplay\0"
    "concavehull_DisplayAll\0concavehull_HideAll\0"
    "stemCurvature\0stemCurvatureDisplay\0"
    "stemCurvature_DisplayAll\0stemCurvature_HideAll\0"
    "stemCurvatureExport\0dbhCloudEdit\0"
    "dbhCloudStopEdit\0reconstruction\0"
    "set_CrownManual\0CrownManualStop\0"
    "set_CrownAutomatic\0crownDisplay\0"
    "crown_DisplayAll\0crown_HideAll\0"
    "CrownHeightDisplay\0crownHeightsDisplayAll\0"
    "crownHeightsHideAll\0crownPositionDisplay\0"
    "crownPositionDisplayAll\0crownPositionHideAll\0"
    "setSectionsVolumeSurfacePosition\0"
    "crownSurfaceBySectionsHideAll\0"
    "crownSurfaceBySectionsDisplayAll\0"
    "crownSurfaceBySectionsDisplayName\0"
    "create3DConvexull\0crownSurface3DHullDisplayAll\0"
    "crownSurface3DHullHideAll\0"
    "crownSurface3DHullDisplayName\0"
    "crownExternalPtsDisplayAll\0"
    "crownExternalPtsHideAll\0"
    "computeCrownsIntersections\0"
    "intersectionsShowAll\0intersectionsHideAll\0"
    "showCrownIntersectionsTable\0"
    "exportCrownAttributes\0exportIntersections\0"
    "recomputeAfterTreePosChenge\0"
    "crownVolumeByVoxels\0eraseSelectedClouds\0"
    "mergeCloudsByID\0labelClouds\0labelCloudsOFF\0"
    "mergeClouds\0minusCloud\0splitCloud\0"
    "voxelize\0accuracy\0duplicatePoints\0"
    "topView\0bottomView\0frontView\0backView\0"
    "sideAView\0sideBView\0perspective\0ortho\0"
    "set_ConcaveCloud\0set_ConvexCloud\0"
    "save_tiff\0bgColor\0about\0dispCloud\0"
    "removeCloud\0deleteCloud\0colorCloud\0"
    "colorCloudField\0PointSize\0undo\0"
    "displayHideEditCloud\0saveVegetation\0"
    "Cloud*\0saveTerrain\0saveTree\0saveRest\0"
    "c\0showProgressBar100percent\0"
    "showProgressBarAt\0QProgressBar*\0pBar\0"
    "a\0showProgressBarInfinity\0createPBar\0"
    "showPBarValue\0removePbar\0nextSlice\0"
    "prevSlice\0slice\0sliceStop\0refreshAttTable"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
     151,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,  769,    2, 0x06 /* Public */,
       3,    0,  770,    2, 0x06 /* Public */,
       4,    0,  771,    2, 0x06 /* Public */,
       5,    0,  772,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    0,  773,    2, 0x08 /* Private */,
       7,    0,  774,    2, 0x08 /* Private */,
       8,    0,  775,    2, 0x08 /* Private */,
       9,    0,  776,    2, 0x08 /* Private */,
      10,    0,  777,    2, 0x08 /* Private */,
      11,    0,  778,    2, 0x08 /* Private */,
      12,    0,  779,    2, 0x08 /* Private */,
      13,    0,  780,    2, 0x08 /* Private */,
      14,    0,  781,    2, 0x08 /* Private */,
      15,    0,  782,    2, 0x08 /* Private */,
      16,    0,  783,    2, 0x08 /* Private */,
      17,    0,  784,    2, 0x08 /* Private */,
      18,    0,  785,    2, 0x08 /* Private */,
      19,    0,  786,    2, 0x08 /* Private */,
      20,    1,  787,    2, 0x08 /* Private */,
      23,    0,  790,    2, 0x08 /* Private */,
      24,    0,  791,    2, 0x08 /* Private */,
      25,    0,  792,    2, 0x08 /* Private */,
      26,    0,  793,    2, 0x08 /* Private */,
      27,    0,  794,    2, 0x08 /* Private */,
      28,    0,  795,    2, 0x08 /* Private */,
      29,    0,  796,    2, 0x08 /* Private */,
      30,    0,  797,    2, 0x08 /* Private */,
      31,    0,  798,    2, 0x08 /* Private */,
      32,    0,  799,    2, 0x08 /* Private */,
      33,    0,  800,    2, 0x08 /* Private */,
      34,    0,  801,    2, 0x08 /* Private */,
      35,    0,  802,    2, 0x08 /* Private */,
      36,    0,  803,    2, 0x08 /* Private */,
      37,    0,  804,    2, 0x08 /* Private */,
      38,    1,  805,    2, 0x08 /* Private */,
      40,    0,  808,    2, 0x08 /* Private */,
      41,    0,  809,    2, 0x08 /* Private */,
      42,    0,  810,    2, 0x08 /* Private */,
      43,    0,  811,    2, 0x08 /* Private */,
      44,    1,  812,    2, 0x08 /* Private */,
      45,    0,  815,    2, 0x08 /* Private */,
      46,    0,  816,    2, 0x08 /* Private */,
      47,    0,  817,    2, 0x08 /* Private */,
      48,    1,  818,    2, 0x08 /* Private */,
      49,    0,  821,    2, 0x08 /* Private */,
      50,    0,  822,    2, 0x08 /* Private */,
      51,    0,  823,    2, 0x08 /* Private */,
      52,    0,  824,    2, 0x08 /* Private */,
      53,    1,  825,    2, 0x08 /* Private */,
      54,    0,  828,    2, 0x08 /* Private */,
      55,    0,  829,    2, 0x08 /* Private */,
      56,    0,  830,    2, 0x08 /* Private */,
      57,    1,  831,    2, 0x08 /* Private */,
      58,    0,  834,    2, 0x08 /* Private */,
      59,    0,  835,    2, 0x08 /* Private */,
      60,    0,  836,    2, 0x08 /* Private */,
      61,    1,  837,    2, 0x08 /* Private */,
      62,    0,  840,    2, 0x08 /* Private */,
      63,    0,  841,    2, 0x08 /* Private */,
      64,    0,  842,    2, 0x08 /* Private */,
      65,    1,  843,    2, 0x08 /* Private */,
      66,    0,  846,    2, 0x08 /* Private */,
      67,    0,  847,    2, 0x08 /* Private */,
      68,    0,  848,    2, 0x08 /* Private */,
      69,    1,  849,    2, 0x08 /* Private */,
      70,    0,  852,    2, 0x08 /* Private */,
      71,    0,  853,    2, 0x08 /* Private */,
      72,    0,  854,    2, 0x08 /* Private */,
      73,    1,  855,    2, 0x08 /* Private */,
      74,    0,  858,    2, 0x08 /* Private */,
      75,    0,  859,    2, 0x08 /* Private */,
      76,    0,  860,    2, 0x08 /* Private */,
      77,    0,  861,    2, 0x08 /* Private */,
      78,    0,  862,    2, 0x08 /* Private */,
      79,    0,  863,    2, 0x08 /* Private */,
      80,    0,  864,    2, 0x08 /* Private */,
      81,    0,  865,    2, 0x08 /* Private */,
      82,    0,  866,    2, 0x08 /* Private */,
      83,    1,  867,    2, 0x08 /* Private */,
      84,    0,  870,    2, 0x08 /* Private */,
      85,    0,  871,    2, 0x08 /* Private */,
      86,    1,  872,    2, 0x08 /* Private */,
      87,    0,  875,    2, 0x08 /* Private */,
      88,    0,  876,    2, 0x08 /* Private */,
      89,    1,  877,    2, 0x08 /* Private */,
      90,    0,  880,    2, 0x08 /* Private */,
      91,    0,  881,    2, 0x08 /* Private */,
      92,    0,  882,    2, 0x08 /* Private */,
      93,    0,  883,    2, 0x08 /* Private */,
      94,    0,  884,    2, 0x08 /* Private */,
      95,    1,  885,    2, 0x08 /* Private */,
      96,    0,  888,    2, 0x08 /* Private */,
      97,    0,  889,    2, 0x08 /* Private */,
      98,    0,  890,    2, 0x08 /* Private */,
      99,    1,  891,    2, 0x08 /* Private */,
     100,    0,  894,    2, 0x08 /* Private */,
     101,    0,  895,    2, 0x08 /* Private */,
     102,    0,  896,    2, 0x08 /* Private */,
     103,    0,  897,    2, 0x08 /* Private */,
     104,    0,  898,    2, 0x08 /* Private */,
     105,    0,  899,    2, 0x08 /* Private */,
     106,    0,  900,    2, 0x08 /* Private */,
     107,    0,  901,    2, 0x08 /* Private */,
     108,    0,  902,    2, 0x08 /* Private */,
     109,    0,  903,    2, 0x08 /* Private */,
     110,    0,  904,    2, 0x08 /* Private */,
     111,    0,  905,    2, 0x08 /* Private */,
     112,    0,  906,    2, 0x08 /* Private */,
     113,    0,  907,    2, 0x08 /* Private */,
     114,    0,  908,    2, 0x08 /* Private */,
     115,    0,  909,    2, 0x08 /* Private */,
     116,    0,  910,    2, 0x08 /* Private */,
     117,    0,  911,    2, 0x08 /* Private */,
     118,    0,  912,    2, 0x08 /* Private */,
     119,    0,  913,    2, 0x08 /* Private */,
     120,    0,  914,    2, 0x08 /* Private */,
     121,    0,  915,    2, 0x08 /* Private */,
     122,    0,  916,    2, 0x08 /* Private */,
     123,    0,  917,    2, 0x08 /* Private */,
     124,    0,  918,    2, 0x08 /* Private */,
     125,    0,  919,    2, 0x08 /* Private */,
     126,    0,  920,    2, 0x08 /* Private */,
     127,    0,  921,    2, 0x08 /* Private */,
     128,    0,  922,    2, 0x08 /* Private */,
     129,    0,  923,    2, 0x08 /* Private */,
     130,    0,  924,    2, 0x08 /* Private */,
     131,    0,  925,    2, 0x08 /* Private */,
     132,    0,  926,    2, 0x08 /* Private */,
     133,    1,  927,    2, 0x08 /* Private */,
     134,    1,  930,    2, 0x08 /* Private */,
     135,    1,  933,    2, 0x08 /* Private */,
     136,    1,  936,    2, 0x08 /* Private */,
     137,    1,  939,    2, 0x08 /* Private */,
     138,    1,  942,    2, 0x08 /* Private */,
     139,    0,  945,    2, 0x08 /* Private */,
     140,    0,  946,    2, 0x08 /* Private */,
     141,    1,  947,    2, 0x08 /* Private */,
     143,    1,  950,    2, 0x08 /* Private */,
     144,    1,  953,    2, 0x08 /* Private */,
     145,    1,  956,    2, 0x08 /* Private */,
     147,    0,  959,    2, 0x08 /* Private */,
     148,    2,  960,    2, 0x08 /* Private */,
     152,    0,  965,    2, 0x08 /* Private */,
     153,    0,  966,    2, 0x08 /* Private */,
     154,    1,  967,    2, 0x08 /* Private */,
     155,    0,  970,    2, 0x08 /* Private */,
     156,    0,  971,    2, 0x08 /* Private */,
     157,    0,  972,    2, 0x08 /* Private */,
     158,    0,  973,    2, 0x08 /* Private */,
     159,    0,  974,    2, 0x08 /* Private */,
     160,    0,  975,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::QString,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 21,   22,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void, QMetaType::QString,   39,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 142,    2,
    QMetaType::Void, 0x80000000 | 142,    2,
    QMetaType::Void, 0x80000000 | 142,    2,
    QMetaType::Void, 0x80000000 | 142,  146,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 149, QMetaType::Int,  150,  151,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        MainWindow *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->savedVege(); break;
        case 1: _t->savedTerrain(); break;
        case 2: _t->savedTree(); break;
        case 3: _t->savedRest(); break;
        case 4: { QString _r = _t->get_path();
            if (_a[0]) *reinterpret_cast< QString*>(_a[0]) = _r; }  break;
        case 5: _t->newProject(); break;
        case 6: _t->openProject(); break;
        case 7: _t->closeProject(); break;
        case 8: _t->importProject(); break;
        case 9: _t->showAttributeTable(); break;
        case 10: _t->importTerrainFile(); break;
        case 11: _t->importBaseCloud(); break;
        case 12: _t->importVegeCloud(); break;
        case 13: _t->importTreeCloud(); break;
        case 14: _t->importOstCloud(); break;
        case 15: _t->exportCloud(); break;
        case 16: _t->exportConvexTxt(); break;
        case 17: _t->exportConcaveTxt(); break;
        case 18: _t->closeEvent((*reinterpret_cast< QCloseEvent*(*)>(_a[1]))); break;
        case 19: _t->voxelgrid(); break;
        case 20: _t->octreeSlot(); break;
        case 21: _t->manualAdjust(); break;
        case 22: _t->manualAdjustStop(); break;
        case 23: _t->IDWslot(); break;
        case 24: _t->statisticalOutlierRemoval(); break;
        case 25: _t->radiusOutlierRemoval(); break;
        case 26: _t->manualSelect(); break;
        case 27: _t->manualSelectStop(); break;
        case 28: _t->segmentation(); break;
        case 29: _t->manualSelectExit(); break;
        case 30: _t->treeEdit(); break;
        case 31: _t->treeEditStop(); break;
        case 32: _t->treeAtributes(); break;
        case 33: _t->dbhHT(); break;
        case 34: _t->dbhHTDisplay((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 35: _t->dbhHT_DisplayAll(); break;
        case 36: _t->dbhHT_HideAll(); break;
        case 37: _t->dbhLSR(); break;
        case 38: _t->dbhCheck(); break;
        case 39: _t->dbhLSRDisplay((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 40: _t->dbhLSR_DisplayAll(); break;
        case 41: _t->dbhLSR_HideAll(); break;
        case 42: _t->height(); break;
        case 43: _t->heightDisplay((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 44: _t->height_DisplayAll(); break;
        case 45: _t->height_HideAll(); break;
        case 46: _t->position(); break;
        case 47: _t->positionHT(); break;
        case 48: _t->positionDisplay((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 49: _t->position_DisplayAll(); break;
        case 50: _t->position_HideAll(); break;
        case 51: _t->length(); break;
        case 52: _t->lengthDisplay((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 53: _t->length_DisplayAll(); break;
        case 54: _t->length_HideAll(); break;
        case 55: _t->skeleton(); break;
        case 56: _t->skeletonDisplay((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 57: _t->skeleton_DisplayAll(); break;
        case 58: _t->skeleton_HideAll(); break;
        case 59: _t->convexhull(); break;
        case 60: _t->convexhullDisplay((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 61: _t->convexhull_DisplayAll(); break;
        case 62: _t->convexhull_HideAll(); break;
        case 63: _t->concavehull(); break;
        case 64: _t->concavehullDisplay((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 65: _t->concavehull_DisplayAll(); break;
        case 66: _t->concavehull_HideAll(); break;
        case 67: _t->stemCurvature(); break;
        case 68: _t->stemCurvatureDisplay((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 69: _t->stemCurvature_DisplayAll(); break;
        case 70: _t->stemCurvature_HideAll(); break;
        case 71: _t->stemCurvatureExport(); break;
        case 72: _t->dbhCloudEdit(); break;
        case 73: _t->dbhCloudStopEdit(); break;
        case 74: _t->reconstruction(); break;
        case 75: _t->set_CrownManual(); break;
        case 76: _t->CrownManualStop(); break;
        case 77: _t->set_CrownAutomatic(); break;
        case 78: _t->crownDisplay((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 79: _t->crown_DisplayAll(); break;
        case 80: _t->crown_HideAll(); break;
        case 81: _t->CrownHeightDisplay((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 82: _t->crownHeightsDisplayAll(); break;
        case 83: _t->crownHeightsHideAll(); break;
        case 84: _t->crownPositionDisplay((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 85: _t->crownPositionDisplayAll(); break;
        case 86: _t->crownPositionHideAll(); break;
        case 87: _t->setSectionsVolumeSurfacePosition(); break;
        case 88: _t->crownSurfaceBySectionsHideAll(); break;
        case 89: _t->crownSurfaceBySectionsDisplayAll(); break;
        case 90: _t->crownSurfaceBySectionsDisplayName((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 91: _t->create3DConvexull(); break;
        case 92: _t->crownSurface3DHullDisplayAll(); break;
        case 93: _t->crownSurface3DHullHideAll(); break;
        case 94: _t->crownSurface3DHullDisplayName((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 95: _t->crownExternalPtsDisplayAll(); break;
        case 96: _t->crownExternalPtsHideAll(); break;
        case 97: _t->computeCrownsIntersections(); break;
        case 98: _t->intersectionsShowAll(); break;
        case 99: _t->intersectionsHideAll(); break;
        case 100: _t->showCrownIntersectionsTable(); break;
        case 101: _t->exportCrownAttributes(); break;
        case 102: _t->exportIntersections(); break;
        case 103: _t->recomputeAfterTreePosChenge(); break;
        case 104: _t->crownVolumeByVoxels(); break;
        case 105: _t->eraseSelectedClouds(); break;
        case 106: _t->mergeCloudsByID(); break;
        case 107: _t->labelClouds(); break;
        case 108: _t->labelCloudsOFF(); break;
        case 109: _t->mergeClouds(); break;
        case 110: _t->minusCloud(); break;
        case 111: _t->splitCloud(); break;
        case 112: _t->voxelize(); break;
        case 113: _t->accuracy(); break;
        case 114: _t->duplicatePoints(); break;
        case 115: _t->topView(); break;
        case 116: _t->bottomView(); break;
        case 117: _t->frontView(); break;
        case 118: _t->backView(); break;
        case 119: _t->sideAView(); break;
        case 120: _t->sideBView(); break;
        case 121: _t->perspective(); break;
        case 122: _t->ortho(); break;
        case 123: _t->set_ConcaveCloud(); break;
        case 124: _t->set_ConvexCloud(); break;
        case 125: _t->save_tiff(); break;
        case 126: _t->bgColor(); break;
        case 127: _t->about(); break;
        case 128: _t->dispCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 129: _t->removeCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 130: _t->deleteCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 131: _t->colorCloud((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 132: _t->colorCloudField((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 133: _t->PointSize((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 134: _t->undo(); break;
        case 135: _t->displayHideEditCloud(); break;
        case 136: _t->saveVegetation((*reinterpret_cast< Cloud*(*)>(_a[1]))); break;
        case 137: _t->saveTerrain((*reinterpret_cast< Cloud*(*)>(_a[1]))); break;
        case 138: _t->saveTree((*reinterpret_cast< Cloud*(*)>(_a[1]))); break;
        case 139: _t->saveRest((*reinterpret_cast< Cloud*(*)>(_a[1]))); break;
        case 140: _t->showProgressBar100percent(); break;
        case 141: _t->showProgressBarAt((*reinterpret_cast< QProgressBar*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 142: _t->showProgressBarInfinity(); break;
        case 143: _t->createPBar(); break;
        case 144: _t->showPBarValue((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 145: _t->removePbar(); break;
        case 146: _t->nextSlice(); break;
        case 147: _t->prevSlice(); break;
        case 148: _t->slice(); break;
        case 149: _t->sliceStop(); break;
        case 150: _t->refreshAttTable(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::savedVege)) {
                *result = 0;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::savedTerrain)) {
                *result = 1;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::savedTree)) {
                *result = 2;
            }
        }
        {
            typedef void (MainWindow::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&MainWindow::savedRest)) {
                *result = 3;
            }
        }
    }
}

const QMetaObject MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_MainWindow.data,
      qt_meta_data_MainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 151)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 151;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 151)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 151;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::savedVege()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void MainWindow::savedTerrain()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}

// SIGNAL 2
void MainWindow::savedTree()
{
    QMetaObject::activate(this, &staticMetaObject, 2, Q_NULLPTR);
}

// SIGNAL 3
void MainWindow::savedRest()
{
    QMetaObject::activate(this, &staticMetaObject, 3, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
