/****************************************************************************
** Meta object code from reading C++ file 'cloud_item_widget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../../src/qt_mainwindow/cloud_item_widget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'cloud_item_widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_CloudItemWidget_t {
    QByteArrayData data[22];
    char stringdata0[374];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_CloudItemWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_CloudItemWidget_t qt_meta_stringdata_CloudItemWidget = {
    {
QT_MOC_LITERAL(0, 0, 15), // "CloudItemWidget"
QT_MOC_LITERAL(1, 16, 20), // "callVisualizeProcess"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 6), // "size_t"
QT_MOC_LITERAL(4, 45, 8), // "cloud_id"
QT_MOC_LITERAL(5, 54, 13), // "CloudProperty"
QT_MOC_LITERAL(6, 68, 8), // "property"
QT_MOC_LITERAL(7, 77, 10), // "Visibility"
QT_MOC_LITERAL(8, 88, 7), // "visible"
QT_MOC_LITERAL(9, 96, 17), // "callCloudItemEdit"
QT_MOC_LITERAL(10, 114, 8), // "TreeList"
QT_MOC_LITERAL(11, 123, 4), // "list"
QT_MOC_LITERAL(12, 128, 13), // "ItemOperation"
QT_MOC_LITERAL(13, 142, 2), // "op"
QT_MOC_LITERAL(14, 145, 8), // "new_name"
QT_MOC_LITERAL(15, 154, 21), // "on_toolButton_clicked"
QT_MOC_LITERAL(16, 176, 34), // "on_actionHideCloudPoints_trig..."
QT_MOC_LITERAL(17, 211, 34), // "on_actionShowCloudPoints_trig..."
QT_MOC_LITERAL(18, 246, 35), // "on_actionHideCloudNormals_tri..."
QT_MOC_LITERAL(19, 282, 35), // "on_actionShowCloudNormals_tri..."
QT_MOC_LITERAL(20, 318, 29), // "on_actionDeleteItem_triggered"
QT_MOC_LITERAL(21, 348, 25) // "on_actionRename_triggered"

    },
    "CloudItemWidget\0callVisualizeProcess\0"
    "\0size_t\0cloud_id\0CloudProperty\0property\0"
    "Visibility\0visible\0callCloudItemEdit\0"
    "TreeList\0list\0ItemOperation\0op\0new_name\0"
    "on_toolButton_clicked\0"
    "on_actionHideCloudPoints_triggered\0"
    "on_actionShowCloudPoints_triggered\0"
    "on_actionHideCloudNormals_triggered\0"
    "on_actionShowCloudNormals_triggered\0"
    "on_actionDeleteItem_triggered\0"
    "on_actionRename_triggered"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_CloudItemWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       3,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    3,   64,    2, 0x06 /* Public */,
       9,    4,   71,    2, 0x06 /* Public */,
       9,    3,   80,    2, 0x26 /* Public | MethodCloned */,

 // slots: name, argc, parameters, tag, flags
      15,    0,   87,    2, 0x08 /* Private */,
      16,    0,   88,    2, 0x08 /* Private */,
      17,    0,   89,    2, 0x08 /* Private */,
      18,    0,   90,    2, 0x08 /* Private */,
      19,    0,   91,    2, 0x08 /* Private */,
      20,    0,   92,    2, 0x08 /* Private */,
      21,    0,   93,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 5, 0x80000000 | 7,    4,    6,    8,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 10, 0x80000000 | 12, QMetaType::QString,    4,   11,   13,   14,
    QMetaType::Void, 0x80000000 | 3, 0x80000000 | 10, 0x80000000 | 12,    4,   11,   13,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void CloudItemWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<CloudItemWidget *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->callVisualizeProcess((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< CloudProperty(*)>(_a[2])),(*reinterpret_cast< Visibility(*)>(_a[3]))); break;
        case 1: _t->callCloudItemEdit((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< TreeList(*)>(_a[2])),(*reinterpret_cast< ItemOperation(*)>(_a[3])),(*reinterpret_cast< QString(*)>(_a[4]))); break;
        case 2: _t->callCloudItemEdit((*reinterpret_cast< size_t(*)>(_a[1])),(*reinterpret_cast< TreeList(*)>(_a[2])),(*reinterpret_cast< ItemOperation(*)>(_a[3]))); break;
        case 3: _t->on_toolButton_clicked(); break;
        case 4: _t->on_actionHideCloudPoints_triggered(); break;
        case 5: _t->on_actionShowCloudPoints_triggered(); break;
        case 6: _t->on_actionHideCloudNormals_triggered(); break;
        case 7: _t->on_actionShowCloudNormals_triggered(); break;
        case 8: _t->on_actionDeleteItem_triggered(); break;
        case 9: _t->on_actionRename_triggered(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (CloudItemWidget::*)(size_t , CloudProperty , Visibility );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CloudItemWidget::callVisualizeProcess)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (CloudItemWidget::*)(size_t , TreeList , ItemOperation , QString );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&CloudItemWidget::callCloudItemEdit)) {
                *result = 1;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject CloudItemWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QWidget::staticMetaObject>(),
    qt_meta_stringdata_CloudItemWidget.data,
    qt_meta_data_CloudItemWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *CloudItemWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *CloudItemWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_CloudItemWidget.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int CloudItemWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
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
void CloudItemWidget::callVisualizeProcess(size_t _t1, CloudProperty _t2, Visibility _t3)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void CloudItemWidget::callCloudItemEdit(size_t _t1, TreeList _t2, ItemOperation _t3, QString _t4)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t2))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t3))), const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t4))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
