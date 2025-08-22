// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from rams_interface:srv/MoveToPose.idl
// generated code does not contain a copyright notice
#include "rams_interface/srv/detail/move_to_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `target`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
rams_interface__srv__MoveToPose_Request__init(rams_interface__srv__MoveToPose_Request * msg)
{
  if (!msg) {
    return false;
  }
  // target
  if (!geometry_msgs__msg__PoseStamped__init(&msg->target)) {
    rams_interface__srv__MoveToPose_Request__fini(msg);
    return false;
  }
  return true;
}

void
rams_interface__srv__MoveToPose_Request__fini(rams_interface__srv__MoveToPose_Request * msg)
{
  if (!msg) {
    return;
  }
  // target
  geometry_msgs__msg__PoseStamped__fini(&msg->target);
}

bool
rams_interface__srv__MoveToPose_Request__are_equal(const rams_interface__srv__MoveToPose_Request * lhs, const rams_interface__srv__MoveToPose_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // target
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->target), &(rhs->target)))
  {
    return false;
  }
  return true;
}

bool
rams_interface__srv__MoveToPose_Request__copy(
  const rams_interface__srv__MoveToPose_Request * input,
  rams_interface__srv__MoveToPose_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // target
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->target), &(output->target)))
  {
    return false;
  }
  return true;
}

rams_interface__srv__MoveToPose_Request *
rams_interface__srv__MoveToPose_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rams_interface__srv__MoveToPose_Request * msg = (rams_interface__srv__MoveToPose_Request *)allocator.allocate(sizeof(rams_interface__srv__MoveToPose_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rams_interface__srv__MoveToPose_Request));
  bool success = rams_interface__srv__MoveToPose_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rams_interface__srv__MoveToPose_Request__destroy(rams_interface__srv__MoveToPose_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rams_interface__srv__MoveToPose_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rams_interface__srv__MoveToPose_Request__Sequence__init(rams_interface__srv__MoveToPose_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rams_interface__srv__MoveToPose_Request * data = NULL;

  if (size) {
    data = (rams_interface__srv__MoveToPose_Request *)allocator.zero_allocate(size, sizeof(rams_interface__srv__MoveToPose_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rams_interface__srv__MoveToPose_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rams_interface__srv__MoveToPose_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rams_interface__srv__MoveToPose_Request__Sequence__fini(rams_interface__srv__MoveToPose_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rams_interface__srv__MoveToPose_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rams_interface__srv__MoveToPose_Request__Sequence *
rams_interface__srv__MoveToPose_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rams_interface__srv__MoveToPose_Request__Sequence * array = (rams_interface__srv__MoveToPose_Request__Sequence *)allocator.allocate(sizeof(rams_interface__srv__MoveToPose_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rams_interface__srv__MoveToPose_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rams_interface__srv__MoveToPose_Request__Sequence__destroy(rams_interface__srv__MoveToPose_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rams_interface__srv__MoveToPose_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rams_interface__srv__MoveToPose_Request__Sequence__are_equal(const rams_interface__srv__MoveToPose_Request__Sequence * lhs, const rams_interface__srv__MoveToPose_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rams_interface__srv__MoveToPose_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rams_interface__srv__MoveToPose_Request__Sequence__copy(
  const rams_interface__srv__MoveToPose_Request__Sequence * input,
  rams_interface__srv__MoveToPose_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rams_interface__srv__MoveToPose_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rams_interface__srv__MoveToPose_Request * data =
      (rams_interface__srv__MoveToPose_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rams_interface__srv__MoveToPose_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rams_interface__srv__MoveToPose_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rams_interface__srv__MoveToPose_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
#include "rosidl_runtime_c/string_functions.h"

bool
rams_interface__srv__MoveToPose_Response__init(rams_interface__srv__MoveToPose_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    rams_interface__srv__MoveToPose_Response__fini(msg);
    return false;
  }
  return true;
}

void
rams_interface__srv__MoveToPose_Response__fini(rams_interface__srv__MoveToPose_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
rams_interface__srv__MoveToPose_Response__are_equal(const rams_interface__srv__MoveToPose_Response * lhs, const rams_interface__srv__MoveToPose_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
rams_interface__srv__MoveToPose_Response__copy(
  const rams_interface__srv__MoveToPose_Response * input,
  rams_interface__srv__MoveToPose_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

rams_interface__srv__MoveToPose_Response *
rams_interface__srv__MoveToPose_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rams_interface__srv__MoveToPose_Response * msg = (rams_interface__srv__MoveToPose_Response *)allocator.allocate(sizeof(rams_interface__srv__MoveToPose_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(rams_interface__srv__MoveToPose_Response));
  bool success = rams_interface__srv__MoveToPose_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
rams_interface__srv__MoveToPose_Response__destroy(rams_interface__srv__MoveToPose_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    rams_interface__srv__MoveToPose_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
rams_interface__srv__MoveToPose_Response__Sequence__init(rams_interface__srv__MoveToPose_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rams_interface__srv__MoveToPose_Response * data = NULL;

  if (size) {
    data = (rams_interface__srv__MoveToPose_Response *)allocator.zero_allocate(size, sizeof(rams_interface__srv__MoveToPose_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = rams_interface__srv__MoveToPose_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        rams_interface__srv__MoveToPose_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
rams_interface__srv__MoveToPose_Response__Sequence__fini(rams_interface__srv__MoveToPose_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      rams_interface__srv__MoveToPose_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

rams_interface__srv__MoveToPose_Response__Sequence *
rams_interface__srv__MoveToPose_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  rams_interface__srv__MoveToPose_Response__Sequence * array = (rams_interface__srv__MoveToPose_Response__Sequence *)allocator.allocate(sizeof(rams_interface__srv__MoveToPose_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = rams_interface__srv__MoveToPose_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
rams_interface__srv__MoveToPose_Response__Sequence__destroy(rams_interface__srv__MoveToPose_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    rams_interface__srv__MoveToPose_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
rams_interface__srv__MoveToPose_Response__Sequence__are_equal(const rams_interface__srv__MoveToPose_Response__Sequence * lhs, const rams_interface__srv__MoveToPose_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!rams_interface__srv__MoveToPose_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
rams_interface__srv__MoveToPose_Response__Sequence__copy(
  const rams_interface__srv__MoveToPose_Response__Sequence * input,
  rams_interface__srv__MoveToPose_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(rams_interface__srv__MoveToPose_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rams_interface__srv__MoveToPose_Response * data =
      (rams_interface__srv__MoveToPose_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!rams_interface__srv__MoveToPose_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          rams_interface__srv__MoveToPose_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!rams_interface__srv__MoveToPose_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
