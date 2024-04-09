/****************************************************************************
** Meta object code from reading C++ file 'vtk_widget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../../src/SubWidget/vtk_widget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'vtk_widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_VTKWidget_t {
    QByteArrayData data[9];
    char stringdata0[99];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_VTKWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_VTKWidget_t qt_meta_stringdata_VTKWidget = {
    {
QT_MOC_LITERAL(0, 0, 9), // "VTKWidget"
QT_MOC_LITERAL(1, 10, 19), // "renderCloudProperty"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 9), // "cloud_obj"
QT_MOC_LITERAL(4, 41, 13), // "CloudProperty"
QT_MOC_LITERAL(5, 55, 8), // "property"
QT_MOC_LITERAL(6, 64, 10), // "Visibility"
QT_MOC_LITERAL(7, 75, 7), // "visible"
QT_MOC_LITERAL(8, 83, 15) // "renderCloudMesh"

    },
    "VTKWidget\0renderCloudProperty\0\0cloud_obj\0"
    "CloudProperty\0property\0Visibility\0"
    "visible\0renderCloudMesh"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_VTKWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    3,   24,    2, 0x0a /* Public */,
       8,    0,   31,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::QVariant, 0x80000000 | 4, 0x80000000 | 6,    3,    5,    7,
    QMetaType::Void,

       0        // eod
};

void VTKWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<VTKWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->renderCloudProperty((*reinterpret_cast< QVariant(*)>(_a[1])),(*reinterpret_cast< CloudProperty(*)>(_a[2])),(*reinterpret_cast< Visibility(*)>(_a[3]))); break;
        case 1: _t->renderCloudMesh(); break;
        default: ;
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject VTKWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_VTKWidget.data,
    qt_meta_data_VTKWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *VTKWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *VTKWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_VTKWidget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int VTKWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
