/*
 * Copyright (c) 2019-present Samsung Electronics Co., Ltd
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
 *  USA
 */

#ifndef __EscargotIteratorOperations__
#define __EscargotIteratorOperations__

#include "runtime/Value.h"

namespace Escargot {

class ExecutionState;

Value getIterator(ExecutionState& state, const Value& obj, const Value& method = Value(Value::EmptyValue));
Value iteratorNext(ExecutionState& state, const Value& iterator, const Value& value = Value(Value::EmptyValue));
bool iteratorComplete(ExecutionState& state, const Value& iterResult);
Value iteratorValue(ExecutionState& state, const Value& iterResult);
Value iteratorStep(ExecutionState& state, const Value& iterator);
Value iteratorClose(ExecutionState& state, const Value& iterator, const Value& completionValue, bool hasThrowOnCompletionType);
Value createIterResultObject(ExecutionState& state, const Value& value, bool done);
}
#endif
