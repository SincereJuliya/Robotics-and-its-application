// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from motion_planner_msgs:srv/ValidatePath.idl
// generated code does not contain a copyright notice

#ifndef MOTION_PLANNER_MSGS__SRV__DETAIL__VALIDATE_PATH__FUNCTIONS_H_
#define MOTION_PLANNER_MSGS__SRV__DETAIL__VALIDATE_PATH__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "motion_planner_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "motion_planner_msgs/srv/detail/validate_path__struct.h"

/// Initialize srv/ValidatePath message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * motion_planner_msgs__srv__ValidatePath_Request
 * )) before or use
 * motion_planner_msgs__srv__ValidatePath_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
bool
motion_planner_msgs__srv__ValidatePath_Request__init(motion_planner_msgs__srv__ValidatePath_Request * msg);

/// Finalize srv/ValidatePath message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
void
motion_planner_msgs__srv__ValidatePath_Request__fini(motion_planner_msgs__srv__ValidatePath_Request * msg);

/// Create srv/ValidatePath message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * motion_planner_msgs__srv__ValidatePath_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
motion_planner_msgs__srv__ValidatePath_Request *
motion_planner_msgs__srv__ValidatePath_Request__create();

/// Destroy srv/ValidatePath message.
/**
 * It calls
 * motion_planner_msgs__srv__ValidatePath_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
void
motion_planner_msgs__srv__ValidatePath_Request__destroy(motion_planner_msgs__srv__ValidatePath_Request * msg);

/// Check for srv/ValidatePath message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
bool
motion_planner_msgs__srv__ValidatePath_Request__are_equal(const motion_planner_msgs__srv__ValidatePath_Request * lhs, const motion_planner_msgs__srv__ValidatePath_Request * rhs);

/// Copy a srv/ValidatePath message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
bool
motion_planner_msgs__srv__ValidatePath_Request__copy(
  const motion_planner_msgs__srv__ValidatePath_Request * input,
  motion_planner_msgs__srv__ValidatePath_Request * output);

/// Initialize array of srv/ValidatePath messages.
/**
 * It allocates the memory for the number of elements and calls
 * motion_planner_msgs__srv__ValidatePath_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
bool
motion_planner_msgs__srv__ValidatePath_Request__Sequence__init(motion_planner_msgs__srv__ValidatePath_Request__Sequence * array, size_t size);

/// Finalize array of srv/ValidatePath messages.
/**
 * It calls
 * motion_planner_msgs__srv__ValidatePath_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
void
motion_planner_msgs__srv__ValidatePath_Request__Sequence__fini(motion_planner_msgs__srv__ValidatePath_Request__Sequence * array);

/// Create array of srv/ValidatePath messages.
/**
 * It allocates the memory for the array and calls
 * motion_planner_msgs__srv__ValidatePath_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
motion_planner_msgs__srv__ValidatePath_Request__Sequence *
motion_planner_msgs__srv__ValidatePath_Request__Sequence__create(size_t size);

/// Destroy array of srv/ValidatePath messages.
/**
 * It calls
 * motion_planner_msgs__srv__ValidatePath_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
void
motion_planner_msgs__srv__ValidatePath_Request__Sequence__destroy(motion_planner_msgs__srv__ValidatePath_Request__Sequence * array);

/// Check for srv/ValidatePath message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
bool
motion_planner_msgs__srv__ValidatePath_Request__Sequence__are_equal(const motion_planner_msgs__srv__ValidatePath_Request__Sequence * lhs, const motion_planner_msgs__srv__ValidatePath_Request__Sequence * rhs);

/// Copy an array of srv/ValidatePath messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
bool
motion_planner_msgs__srv__ValidatePath_Request__Sequence__copy(
  const motion_planner_msgs__srv__ValidatePath_Request__Sequence * input,
  motion_planner_msgs__srv__ValidatePath_Request__Sequence * output);

/// Initialize srv/ValidatePath message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * motion_planner_msgs__srv__ValidatePath_Response
 * )) before or use
 * motion_planner_msgs__srv__ValidatePath_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
bool
motion_planner_msgs__srv__ValidatePath_Response__init(motion_planner_msgs__srv__ValidatePath_Response * msg);

/// Finalize srv/ValidatePath message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
void
motion_planner_msgs__srv__ValidatePath_Response__fini(motion_planner_msgs__srv__ValidatePath_Response * msg);

/// Create srv/ValidatePath message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * motion_planner_msgs__srv__ValidatePath_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
motion_planner_msgs__srv__ValidatePath_Response *
motion_planner_msgs__srv__ValidatePath_Response__create();

/// Destroy srv/ValidatePath message.
/**
 * It calls
 * motion_planner_msgs__srv__ValidatePath_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
void
motion_planner_msgs__srv__ValidatePath_Response__destroy(motion_planner_msgs__srv__ValidatePath_Response * msg);

/// Check for srv/ValidatePath message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
bool
motion_planner_msgs__srv__ValidatePath_Response__are_equal(const motion_planner_msgs__srv__ValidatePath_Response * lhs, const motion_planner_msgs__srv__ValidatePath_Response * rhs);

/// Copy a srv/ValidatePath message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
bool
motion_planner_msgs__srv__ValidatePath_Response__copy(
  const motion_planner_msgs__srv__ValidatePath_Response * input,
  motion_planner_msgs__srv__ValidatePath_Response * output);

/// Initialize array of srv/ValidatePath messages.
/**
 * It allocates the memory for the number of elements and calls
 * motion_planner_msgs__srv__ValidatePath_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
bool
motion_planner_msgs__srv__ValidatePath_Response__Sequence__init(motion_planner_msgs__srv__ValidatePath_Response__Sequence * array, size_t size);

/// Finalize array of srv/ValidatePath messages.
/**
 * It calls
 * motion_planner_msgs__srv__ValidatePath_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
void
motion_planner_msgs__srv__ValidatePath_Response__Sequence__fini(motion_planner_msgs__srv__ValidatePath_Response__Sequence * array);

/// Create array of srv/ValidatePath messages.
/**
 * It allocates the memory for the array and calls
 * motion_planner_msgs__srv__ValidatePath_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
motion_planner_msgs__srv__ValidatePath_Response__Sequence *
motion_planner_msgs__srv__ValidatePath_Response__Sequence__create(size_t size);

/// Destroy array of srv/ValidatePath messages.
/**
 * It calls
 * motion_planner_msgs__srv__ValidatePath_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
void
motion_planner_msgs__srv__ValidatePath_Response__Sequence__destroy(motion_planner_msgs__srv__ValidatePath_Response__Sequence * array);

/// Check for srv/ValidatePath message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
bool
motion_planner_msgs__srv__ValidatePath_Response__Sequence__are_equal(const motion_planner_msgs__srv__ValidatePath_Response__Sequence * lhs, const motion_planner_msgs__srv__ValidatePath_Response__Sequence * rhs);

/// Copy an array of srv/ValidatePath messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_motion_planner_msgs
bool
motion_planner_msgs__srv__ValidatePath_Response__Sequence__copy(
  const motion_planner_msgs__srv__ValidatePath_Response__Sequence * input,
  motion_planner_msgs__srv__ValidatePath_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // MOTION_PLANNER_MSGS__SRV__DETAIL__VALIDATE_PATH__FUNCTIONS_H_
