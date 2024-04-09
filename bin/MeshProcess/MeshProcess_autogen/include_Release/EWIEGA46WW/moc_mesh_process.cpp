/****************************************************************************
** Meta object code from reading C++ file 'mesh_process.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../src/MeshProcess/mesh_process.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mesh_process.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_VtkProcess_t {
    QByteArrayData data[17];
    char stringdata0[192];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_VtkProcess_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_VtkProcess_t qt_meta_stringdata_VtkProcess = {
    {
QT_MOC_LITERAL(0, 0, 10), // "VtkProcess"
QT_MOC_LITERAL(1, 11, 20), // "callVisualizeProcess"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 6), // "size_t"
QT_MOC_LITERAL(4, 40, 8), // "cloud_id"
QT_MOC_LITERAL(5, 49, 13), // "CloudProperty"
QT_MOC_LITERAL(6, 63, 8), // "property"
QT_MOC_LITERAL(7, 72, 10), // "Visibility"
QT_MOC_LITERAL(8, 83, 7), // "visible"
QT_MOC_LITERAL(9, 91, 8), // "TreeList"
QT_MOC_LITERAL(10, 100, 10), // "cloud_type"
QT_MOC_LITERAL(11, 111, 25), // "callCloudItemEditOnScreen"
QT_MOC_LITERAL(12, 137, 4), // "list"
QT_MOC_LITERAL(13, 142, 13), // "ItemOperation"
QT_MOC_LITERAL(14, 156, 2), // "op"
QT_MOC_LITERAL(15, 159, 8), // "new_name"
QT_MOC_LITERAL(16, 168, 23) // "editCloudItemOnDatabase"

    },
    "VtkProcess\0callVisualizeProcess\0\0"
    "size_t\0cloud_id\0CloudProperty\0property\0"
    "Visibility\0visible\0TreeList\0cloud_type\0"
    "callCloudItemEditOnScreen\0list\0"
    "ItemOperation\0op\0new_name\0"
    "editCloudItemOnDatabase"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_VtkProcess[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       5,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    4,   49,    2, 0x06 /* Public */,
       1,    3,   58,    2, 0x26 /* Public | MethodCloned */,
       1,    2,   65,    2, 0x26 /* Public | MethodCloned */,
      11,    4,   70,    2, 0x06 /* Public */,
      11,    3,   79,    2, 0x26 /* Public | MethodCloned */,

 // slots: name, argc, parameters, tag, flags
      16,    3,   86,    2, 0x0a /* Public */,
      16,    2,   93,    2, 0x2a /* Public | MethodCloned */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5, 0x80000000 | 7, 0x80000000 | 9,    4,    6,    8,   10,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5, 0x80000000 | 7,    4,    6,    8,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5,    4,    6,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 9, 0x80000000 | 13, QMetaType::QString,    4,   12,   14,   15,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 9, 0x80000000 | 13,    4,   12,   14,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 13, QMetaType::QString,    4,   14,   15,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 13,    4,   14,

       0        // eod
};

void VtkProcess::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<VtkProcess *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->callVisualizeProcess((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< CloudProperty(*)>(_a[2])),(*reinterpret_cast< Visibility(*)>(_a[3])),(*reinterpret_cast< TreeList(*)>(_a[4]))); break;
        case 1: _t->callVisualizeProcess((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< CloudProperty(*)>(_a[2])),(*reinterpret_cast< Visibility(*)>(_a[3]))); break;
        case 2: _t->callVisualizeProcess((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< CloudProperty(*)>(_a[2]))); break;
        case 3: _t->callCloudItemEditOnScreen((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< TreeList(*)>(_a[2])),(*reinterpret_cast< ItemOperation(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 4: _t->callCloudItemEditOnScreen((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< TreeList(*)>(_a[2])),(*reinterpret_cast< ItemOperation(*)>(_a[3]))); break;
        case 5: _t->editCloudItemOnDatabase((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< ItemOperation(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 6: _t->editCloudItemOnDatabase((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< ItemOperation(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (VtkProcess::*)(size_t , CloudProperty , Visibility , TreeList );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&VtkProcess::callVisualizeProcess)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (VtkProcess::*)(size_t , TreeList , ItemOperation , QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&VtkProcess::callCloudItemEditOnScreen)) {
                *result = 3;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject VtkProcess::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_VtkProcess.data,
    qt_meta_data_VtkProcess,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *VtkProcess::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *VtkProcess::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_VtkProcess.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int VtkProcess::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}

// SIGNAL 0
void VtkProcess::callVisualizeProcess(size_t _t1, CloudProperty _t2, Visibility _t3, TreeList _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 3
void VtkProcess::callCloudItemEditOnScreen(size_t _t1, TreeList _t2, ItemOperation _t3, QString _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
