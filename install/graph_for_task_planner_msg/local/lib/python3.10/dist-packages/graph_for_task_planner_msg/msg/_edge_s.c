// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from graph_for_task_planner_msg:msg/Edge.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "graph_for_task_planner_msg/msg/detail/edge__struct.h"
#include "graph_for_task_planner_msg/msg/detail/edge__functions.h"

bool graph_for_task_planner_msg__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * graph_for_task_planner_msg__msg__point__convert_to_py(void * raw_ros_message);
bool graph_for_task_planner_msg__msg__point__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * graph_for_task_planner_msg__msg__point__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool graph_for_task_planner_msg__msg__edge__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[42];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("graph_for_task_planner_msg.msg._edge.Edge", full_classname_dest, 41) == 0);
  }
  graph_for_task_planner_msg__msg__Edge * ros_message = _ros_message;
  {  // start_point
    PyObject * field = PyObject_GetAttrString(_pymsg, "start_point");
    if (!field) {
      return false;
    }
    if (!graph_for_task_planner_msg__msg__point__convert_from_py(field, &ros_message->start_point)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // end_point
    PyObject * field = PyObject_GetAttrString(_pymsg, "end_point");
    if (!field) {
      return false;
    }
    if (!graph_for_task_planner_msg__msg__point__convert_from_py(field, &ros_message->end_point)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * graph_for_task_planner_msg__msg__edge__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Edge */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("graph_for_task_planner_msg.msg._edge");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Edge");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  graph_for_task_planner_msg__msg__Edge * ros_message = (graph_for_task_planner_msg__msg__Edge *)raw_ros_message;
  {  // start_point
    PyObject * field = NULL;
    field = graph_for_task_planner_msg__msg__point__convert_to_py(&ros_message->start_point);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "start_point", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // end_point
    PyObject * field = NULL;
    field = graph_for_task_planner_msg__msg__point__convert_to_py(&ros_message->end_point);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "end_point", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
