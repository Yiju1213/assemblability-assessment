/****************************************************************************
** Meta object code from reading C++ file 'mainwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../src/MainWindow/mainwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_MainWindow_t {
    QByteArrayData data[28];
    char stringdata0[543];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_MainWindow_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_MainWindow_t qt_meta_stringdata_MainWindow = {
    {
QT_MOC_LITERAL(0, 0, 10), // "MainWindow"
QT_MOC_LITERAL(1, 11, 27), // "callCloudItemEditOnDatabase"
QT_MOC_LITERAL(2, 39, 0), // ""
QT_MOC_LITERAL(3, 40, 6), // "size_t"
QT_MOC_LITERAL(4, 47, 8), // "cloud_id"
QT_MOC_LITERAL(5, 56, 13), // "ItemOperation"
QT_MOC_LITERAL(6, 70, 2), // "op"
QT_MOC_LITERAL(7, 73, 8), // "new_name"
QT_MOC_LITERAL(8, 82, 22), // "visualizeCloudProperty"
QT_MOC_LITERAL(9, 105, 13), // "CloudProperty"
QT_MOC_LITERAL(10, 119, 8), // "property"
QT_MOC_LITERAL(11, 128, 10), // "Visibility"
QT_MOC_LITERAL(12, 139, 7), // "visible"
QT_MOC_LITERAL(13, 147, 8), // "TreeList"
QT_MOC_LITERAL(14, 156, 10), // "cloud_type"
QT_MOC_LITERAL(15, 167, 21), // "editCloudItemOnScreen"
QT_MOC_LITERAL(16, 189, 4), // "list"
QT_MOC_LITERAL(17, 194, 13), // "editCloudItem"
QT_MOC_LITERAL(18, 208, 28), // "on_actionOpenFiles_triggered"
QT_MOC_LITERAL(19, 237, 28), // "on_actionSaveFiles_triggered"
QT_MOC_LITERAL(20, 266, 35), // "on_actionNormalEstimation_tri..."
QT_MOC_LITERAL(21, 302, 34), // "on_actionFPFH_Estimation_trig..."
QT_MOC_LITERAL(22, 337, 41), // "on_actionRANSAC_RoughEstimati..."
QT_MOC_LITERAL(23, 379, 30), // "on_actionChangeColor_triggered"
QT_MOC_LITERAL(24, 410, 34), // "on_actionVoxelGridFilter_trig..."
QT_MOC_LITERAL(25, 445, 30), // "on_actionMergeClouds_triggered"
QT_MOC_LITERAL(26, 476, 30), // "on_actionConvertUnit_triggered"
QT_MOC_LITERAL(27, 507, 35) // "on_actionCollideDetection_tri..."

    },
    "MainWindow\0callCloudItemEditOnDatabase\0"
    "\0size_t\0cloud_id\0ItemOperation\0op\0"
    "new_name\0visualizeCloudProperty\0"
    "CloudProperty\0property\0Visibility\0"
    "visible\0TreeList\0cloud_type\0"
    "editCloudItemOnScreen\0list\0editCloudItem\0"
    "on_actionOpenFiles_triggered\0"
    "on_actionSaveFiles_triggered\0"
    "on_actionNormalEstimation_triggered\0"
    "on_actionFPFH_Estimation_triggered\0"
    "on_actionRANSAC_RoughEstimation_triggered\0"
    "on_actionChangeColor_triggered\0"
    "on_actionVoxelGridFilter_triggered\0"
    "on_actionMergeClouds_triggered\0"
    "on_actionConvertUnit_triggered\0"
    "on_actionCollideDetection_triggered"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_MainWindow[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      17,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    3,   99,    2, 0x06 /* Public */,
       1,    2,  106,    2, 0x26 /* Public | MethodCloned */,

 // slots: name, argc, parameters, tag, flags
       8,    4,  111,    2, 0x08 /* Private */,
      15,    4,  120,    2, 0x08 /* Private */,
      15,    3,  129,    2, 0x28 /* Private | MethodCloned */,
      17,    4,  136,    2, 0x08 /* Private */,
      17,    3,  145,    2, 0x28 /* Private | MethodCloned */,
      18,    0,  152,    2, 0x08 /* Private */,
      19,    0,  153,    2, 0x08 /* Private */,
      20,    0,  154,    2, 0x08 /* Private */,
      21,    0,  155,    2, 0x08 /* Private */,
      22,    0,  156,    2, 0x08 /* Private */,
      23,    0,  157,    2, 0x08 /* Private */,
      24,    0,  158,    2, 0x08 /* Private */,
      25,    0,  159,    2, 0x08 /* Private */,
      26,    0,  160,    2, 0x08 /* Private */,
      27,    0,  161,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5, QMetaType::QString,    4,    6,    7,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    6,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 9, 0x80000000 | 11, 0x80000000 | 13,    4,   10,   12,   14,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 13, 0x80000000 | 5, QMetaType::QString,    4,   16,    6,    7,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 13, 0x80000000 | 5,    4,   16,    6,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 13, 0x80000000 | 5, QMetaType::QString,    4,   16,    6,    7,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 13, 0x80000000 | 5,    4,   16,    6,
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

       0        // eod
};

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MainWindow *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->callCloudItemEditOnDatabase((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< ItemOperation(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 1: _t->callCloudItemEditOnDatabase((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< ItemOperation(*)>(_a[2]))); break;
        case 2: _t->visualizeCloudProperty((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< CloudProperty(*)>(_a[2])),(*reinterpret_cast< Visibility(*)>(_a[3])),(*reinterpret_cast< TreeList(*)>(_a[4]))); break;
        case 3: _t->editCloudItemOnScreen((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< TreeList(*)>(_a[2])),(*reinterpret_cast< ItemOperation(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 4: _t->editCloudItemOnScreen((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< TreeList(*)>(_a[2])),(*reinterpret_cast< ItemOperation(*)>(_a[3]))); break;
        case 5: _t->editCloudItem((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< TreeList(*)>(_a[2])),(*reinterpret_cast< ItemOperation(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 6: _t->editCloudItem((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< TreeList(*)>(_a[2])),(*reinterpret_cast< ItemOperation(*)>(_a[3]))); break;
        case 7: _t->on_actionOpenFiles_triggered(); break;
        case 8: _t->on_actionSaveFiles_triggered(); break;
        case 9: _t->on_actionNormalEstimation_triggered(); break;
        case 10: _t->on_actionFPFH_Estimation_triggered(); break;
        case 11: _t->on_actionRANSAC_RoughEstimation_triggered(); break;
        case 12: _t->on_actionChangeColor_triggered(); break;
        case 13: _t->on_actionVoxelGridFilter_triggered(); break;
        case 14: _t->on_actionMergeClouds_triggered(); break;
        case 15: _t->on_actionConvertUnit_triggered(); break;
        case 16: _t->on_actionCollideDetection_triggered(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (MainWindow::*)(size_t , ItemOperation , QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&MainWindow::callCloudItemEditOnDatabase)) {
                *result = 0;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_MainWindow.data,
    qt_meta_data_MainWindow,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_MainWindow.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 17)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 17;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 17)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 17;
    }
    return _id;
}

// SIGNAL 0
void MainWindow::callCloudItemEditOnDatabase(size_t _t1, ItemOperation _t2, QString _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
