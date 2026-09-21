// Standalone regression coverage for separated ObjectStructure index metadata.
function check(actual, expected, message) {
    if (actual !== expected) throw new Error(message + ': ' + actual + ' != ' + expected);
}

// Strings and symbols enter separate storage even when the object never
// acquires an index property.
var plainSymbol1 = Symbol('plain-1');
var plainSymbol2 = Symbol('plain-2');
var plainMixed = {};
plainMixed[plainSymbol1] = 1;
plainMixed.a = 2;
plainMixed[plainSymbol2] = 3;
plainMixed.b = 4;
var plainMixedKeys = Reflect.ownKeys(plainMixed);
check(plainMixedKeys[0], 'a', 'plain mixed first string');
check(plainMixedKeys[1], 'b', 'plain mixed second string');
check(plainMixedKeys[2], plainSymbol1, 'plain mixed first symbol');
check(plainMixedKeys[3], plainSymbol2, 'plain mixed second symbol');
delete plainMixed.a;
delete plainMixed[plainSymbol1];
plainMixed.c = 5;
plainMixed[plainSymbol1] = 6;
plainMixedKeys = Reflect.ownKeys(plainMixed);
check(plainMixedKeys[0], 'b', 'plain partitioned surviving string');
check(plainMixedKeys[1], 'c', 'plain partitioned reinserted string');
check(plainMixedKeys[2], plainSymbol2, 'plain partitioned surviving symbol');
check(plainMixedKeys[3], plainSymbol1, 'plain partitioned reinserted symbol');
check(plainMixed[plainSymbol1], 6, 'plain partitioned symbol value');

// Arguments objects use a prebuilt string/symbol structure. Keep its fixed
// value slots aligned with the partitioned metadata in both mapped and
// unmapped forms.
function mappedArgumentsProbe(value) {
    check(arguments.callee, mappedArgumentsProbe, 'mapped arguments callee slot');
    check(arguments[Symbol.iterator], Array.prototype.values, 'mapped arguments iterator slot');
    check(Array.from(arguments)[0], value, 'mapped arguments iteration');
}
mappedArgumentsProbe(17);
function unmappedArgumentsProbe(value = 23) {
    check(arguments[Symbol.iterator], Array.prototype.values, 'unmapped arguments iterator slot');
    check(Array.from(arguments)[0], value, 'unmapped arguments iteration');
}
unmappedArgumentsProbe(23);

function exercise(count, numeric) {
    var object = Object.create(null);
    function key(i) { return numeric ? String(i) : 'property_' + i; }
    for (var i = 0; i < count; i++) {
        object[key(i)] = i;
        if ((i & 127) === 0) check(object[key(i >> 1)], i >> 1, 'growing lookup');
    }
    for (var i = count - 1; i >= 0; i--) check(object[key(i)], i, 'reverse lookup');
    check(object.missing, undefined, 'cached miss');
    object.missing = 7;
    check(object.missing, 7, 'insert after miss');
    var middle = count >> 1;
    delete object[key(middle)];
    check(object[key(middle)], undefined, 'deleted entry');
    check(object[key(count - 1)], count - 1, 'shifted index');
    check(object[key(count - 2)], count - 2, 'neighbor survives delete count=' + count + ' numeric=' + numeric);
    object[key(middle)] = -1;
    check(object[key(middle)], -1, 'reinsert');
    check(object[key(count - 2)], count - 2, 'index survives reinsertion count=' + count + ' numeric=' + numeric);
    Object.defineProperty(object, key(0), { get: function() { return 99; }, configurable: true });
    check(object[key(0)], 99, 'descriptor replacement');
    check(object[key(count - 2)], count - 2, 'index survives descriptor replacement count=' + count + ' numeric=' + numeric);
    if (typeof gc === 'function') gc();
    check(object[key(count - 2)], count - 2, 'index survives GC count=' + count + ' numeric=' + numeric);
    var keys = Object.keys(object);
    check(keys.length, count + 1, 'enumeration count');
    if (numeric) {
        check(keys[0], '0', 'numeric enumeration start');
        check(keys[count - 1], String(count - 1), 'numeric enumeration end');
    } else {
        check(keys[keys.length - 1], key(middle), 'reinsertion order');
    }
}

[37, 64, 65, 127, 255, 256, 511, 1024, 65535, 65536, 65540].forEach(function(count) {
    exercise(count, false);
    exercise(count, true);
});

var mixed = {};
for (var i = 0; i < 300; i++) mixed[i] = i;
var unusual = ['01', '-0', '-1', '1.5', '4294967294', '4294967295',
               '9007199254740991', '123456789012345678901234567890', '', '한글'];
var symbols = [Symbol('same'), Symbol('same')];
unusual.concat(symbols).forEach(function(key, i) { mixed[key] = 'v' + i; });
for (var round = 0; round < 3; round++) {
    unusual.concat(symbols).forEach(function(key, i) { check(mixed[key], 'v' + i, 'mixed key'); });
    check(mixed[1], 1, 'canonical index distinct from 01');
    if (typeof gc === 'function') gc();
}

// Grow a dense index beyond its initial range, then shrink across thresholds.
for (var i = 300; i < 1200; i++) mixed[i] = i;
for (var i = 1199; i >= 0; i--) check(mixed[i], i, 'dense growth');
for (var i = 1199; i >= 30; i--) delete mixed[i];
for (var i = 0; i < 30; i++) check(mixed[i], i, 'shrink');
unusual.concat(symbols).forEach(function(key, i) { check(mixed[key], 'v' + i, 'shrink mixed'); });

// ObjectPropertyName can carry small transient numeric keys, while
// ObjectStructurePropertyName now always carries a String or Symbol. Verify
// that both transient paths reach the same separated uint32 metadata.
var boundaries = Object.create(null);
var inlineMaximum = 0x3fffffff;
var stringFallback = 0x40000000;
boundaries[inlineMaximum] = 'inline-number';
check(boundaries[String(inlineMaximum)], 'inline-number', 'inline number queried as string');
boundaries[String(inlineMaximum)] = 'inline-string';
check(boundaries[inlineMaximum], 'inline-string', 'inline string queried as number');
boundaries[stringFallback] = 'fallback-number';
check(boundaries[String(stringFallback)], 'fallback-number', 'fallback number queried as string');
boundaries[String(stringFallback)] = 'fallback-string';
check(boundaries[stringFallback], 'fallback-string', 'fallback string queried as number');
Object.defineProperty(boundaries, String(inlineMaximum), { value: 'descriptor', enumerable: true, configurable: true });
check(Object.getOwnPropertyDescriptor(boundaries, inlineMaximum).value, 'descriptor', 'numeric descriptor lookup');
delete boundaries[String(inlineMaximum)];
check(boundaries[inlineMaximum], undefined, 'numeric delete through string');
boundaries[inlineMaximum] = 'reinserted';
var boundaryKeys = Reflect.ownKeys(boundaries);
check(boundaryKeys[0], String(inlineMaximum), 'inline key order and string conversion');
check(boundaryKeys[1], String(stringFallback), 'fallback key order and string conversion');

var canonical = {};
canonical[1] = 'one';
canonical['01'] = 'leading-zero';
canonical['-0'] = 'minus-zero';
canonical['1.5'] = 'fraction';
check(canonical['1'], 'one', 'canonical string lookup');
check(canonical[1], 'one', 'canonical number lookup');
check(canonical['01'], 'leading-zero', 'leading zero remains string');
check(canonical['-0'], 'minus-zero', 'minus zero remains string');
check(canonical['1.5'], 'fraction', 'fraction remains string');
check(Reflect.ownKeys(canonical).join(','), '1,01,-0,1.5', 'canonical own-key order');

// One and two numeric keys stay inline, the third promotes to the packed
// vector, and deleting back to two demotes without disturbing slots or a
// materialized descriptor.
var inlineIndexes = Object.create(null);
inlineIndexes[90] = 'ninety';
inlineIndexes[10] = 'ten';
Object.defineProperty(inlineIndexes, '90', {
    value: 'ninety-custom',
    writable: false,
    enumerable: true,
    configurable: true
});
check(inlineIndexes[10], 'ten', 'inline sorted insertion value');
check(inlineIndexes[90], 'ninety-custom', 'inline custom descriptor value');
inlineIndexes[50] = 'fifty';
check(Reflect.ownKeys(inlineIndexes).join(','), '10,50,90', 'inline-to-vector promotion order');
check(Object.getOwnPropertyDescriptor(inlineIndexes, '90').writable, false, 'descriptor survives promotion');
delete inlineIndexes[50];
check(Reflect.ownKeys(inlineIndexes).join(','), '10,90', 'vector-to-inline demotion order');
check(inlineIndexes[10], 'ten', 'value survives demotion');
check(inlineIndexes[90], 'ninety-custom', 'custom value survives demotion');
check(Object.getOwnPropertyDescriptor(inlineIndexes, '90').writable, false, 'descriptor survives demotion');
delete inlineIndexes[10];
check(inlineIndexes[90], 'ninety-custom', 'single inline key after delete');

// Separated storage keeps values aligned when named properties are added
// after numeric properties and orders the three key domains independently.
var orderedSymbol = Symbol('ordered');
var separated = {};
separated.z = 'z';
separated[7] = 'seven';
separated[2] = 'two';
separated.a = 'a';
separated[4294967294] = 'max-index';
separated[4294967295] = 'not-an-index';
separated[orderedSymbol] = 'symbol';
check(separated.z, 'z', 'named value before indexes');
check(separated.a, 'a', 'named value inserted before index suffix');
check(separated[2], 'two', 'small separated index value');
check(separated[4294967294], 'max-index', 'maximum array index value');
var separatedKeys = Reflect.ownKeys(separated);
check(separatedKeys.slice(0, 6).join(','), '2,7,4294967294,z,a,4294967295', 'separated domain order');
check(separatedKeys[6], orderedSymbol, 'symbols follow strings');

// Symbols are partitioned by identity, not interned by description.
var firstSameSymbol = Symbol('same-description');
var secondSameSymbol = Symbol('same-description');
separated[firstSameSymbol] = 'first-symbol';
separated[secondSameSymbol] = 'second-symbol';
check(separated[firstSameSymbol], 'first-symbol', 'first same-description symbol identity');
check(separated[secondSameSymbol], 'second-symbol', 'second same-description symbol identity');
var symbolKeys = Reflect.ownKeys(separated).filter(function(key) { return typeof key === 'symbol'; });
check(symbolKeys.length, 3, 'partitioned symbol count');
check(symbolKeys[0], orderedSymbol, 'first symbol insertion order');
check(symbolKeys[1], firstSameSymbol, 'same-description first insertion order');
check(symbolKeys[2], secondSameSymbol, 'same-description second insertion order');
delete separated[firstSameSymbol];
separated[firstSameSymbol] = 'first-symbol-reinserted';
symbolKeys = Reflect.ownKeys(separated).filter(function(key) { return typeof key === 'symbol'; });
check(symbolKeys[1], secondSameSymbol, 'symbol survives neighbor deletion');
check(symbolKeys[2], firstSameSymbol, 'reinserted symbol moves to end');
check(separated[firstSameSymbol], 'first-symbol-reinserted', 'reinserted symbol value');

// Adding the first index repartitions an existing mixed string/symbol layout.
// Verify that every pre-existing value follows its key into the new slots.
var transition = {};
var transitionSymbol1 = Symbol('transition');
var transitionSymbol2 = Symbol('transition');
transition[transitionSymbol1] = 'symbol-1';
transition.after = 'string';
transition[transitionSymbol2] = 'symbol-2';
transition[4] = 'index';
check(transition[transitionSymbol1], 'symbol-1', 'first-index transition first symbol value');
check(transition.after, 'string', 'first-index transition string value');
check(transition[transitionSymbol2], 'symbol-2', 'first-index transition second symbol value');
check(transition[4], 'index', 'first-index transition numeric value');
var transitionKeys = Reflect.ownKeys(transition);
check(transitionKeys.slice(0, 2).join(','), '4,after', 'first-index transition string order');
check(transitionKeys[2], transitionSymbol1, 'first-index transition first symbol order');
check(transitionKeys[3], transitionSymbol2, 'first-index transition second symbol order');

delete separated.a;
delete separated[7];
separated.afterDelete = 'after';
separated[7] = 'seven-again';
check(separated.afterDelete, 'after', 'named insertion after mixed deletion');
check(separated[7], 'seven-again', 'index reinsertion after mixed deletion');
check(Reflect.ownKeys(separated).slice(0, 6).join(','), '2,7,4294967294,z,4294967295,afterDelete', 'mixed deletion order');

// Removing the final numeric key retains the independent string/symbol areas.
var finalIndex = {};
var finalIndexSymbol = Symbol('final-index-symbol');
finalIndex.before = 1;
finalIndex[finalIndexSymbol] = 2;
finalIndex[8] = 3;
finalIndex.after = 4;
delete finalIndex[8];
check(finalIndex.before, 1, 'final index removal string before');
check(finalIndex.after, 4, 'final index removal string after');
check(finalIndex[finalIndexSymbol], 2, 'final index removal symbol');
var finalIndexKeys = Reflect.ownKeys(finalIndex);
check(finalIndexKeys.slice(0, 2).join(','), 'before,after', 'final index removal string order');
check(finalIndexKeys[2], finalIndexSymbol, 'final index removal symbol order');

// Cross the linear-to-hash threshold with sparse and high uint32 indexes.
var sparse = Object.create(null);
var sparseIndexes = [4000000000, 3, 70000, 19, 999999999, 0, 65536, 42, 123456789, 4294967294, 256, 4096];
sparseIndexes.forEach(function(index, i) { sparse[index] = 's' + i; });
sparseIndexes.forEach(function(index, i) { check(sparse[index], 's' + i, 'sparse hash lookup'); });
check(Object.keys(sparse).join(','), sparseIndexes.slice().sort(function(a, b) { return a - b; }).join(','), 'sparse numeric order');
Object.defineProperty(sparse, '42', {
    get: function() { return 'accessor'; },
    enumerable: true,
    configurable: true
});
check(sparse[42], 'accessor', 'numeric accessor descriptor');

// Sparse structures keep append-order slots and their ordered ordinal index
// after shrinking below the hash threshold; they cannot demote without also
// moving Object::m_values.
var sparseSurvivors = [4000000000, 3];
sparseIndexes.forEach(function(index) {
    if (sparseSurvivors.indexOf(index) === -1) delete sparse[index];
});
check(sparse[4000000000], 's0', 'sparse survivor high value');
check(sparse[3], 's1', 'sparse survivor low value');
check(Reflect.ownKeys(sparse).join(','), '3,4000000000', 'sparse shrink order');
sparse[17] = 'after-shrink';
check(sparse[17], 'after-shrink', 'sparse add after shrink');
check(sparse[4000000000], 's0', 'sparse high value after regrowth');
check(Reflect.ownKeys(sparse).join(','), '3,17,4000000000', 'sparse regrowth order');

// Default numeric descriptors are implicit. A custom descriptor materializes
// the parallel descriptor area, and restoring/deleting the last exception can
// collapse it back without changing the other numeric properties.
var defaultDescriptors = Object.create(null);
for (var descriptorIndex = 0; descriptorIndex < 300; descriptorIndex++) {
    defaultDescriptors[descriptorIndex * 17] = descriptorIndex;
}
var ordinaryDescriptor = Object.getOwnPropertyDescriptor(defaultDescriptors, '289');
check(ordinaryDescriptor.writable, true, 'implicit descriptor writable');
check(ordinaryDescriptor.enumerable, true, 'implicit descriptor enumerable');
check(ordinaryDescriptor.configurable, true, 'implicit descriptor configurable');
Object.defineProperty(defaultDescriptors, '289', {
    value: 1700,
    writable: false,
    enumerable: false,
    configurable: true
});
defaultDescriptors[9001] = 'added-after-materialize';
check(defaultDescriptors[289], 1700, 'materialized descriptor value');
check(Object.keys(defaultDescriptors).indexOf('289'), -1, 'materialized descriptor enumerable');
ordinaryDescriptor = Object.getOwnPropertyDescriptor(defaultDescriptors, '306');
check(ordinaryDescriptor.writable, true, 'neighbor descriptor writable');
check(ordinaryDescriptor.enumerable, true, 'neighbor descriptor enumerable');
check(ordinaryDescriptor.configurable, true, 'neighbor descriptor configurable');
Object.defineProperty(defaultDescriptors, '289', {
    value: 1701,
    writable: true,
    enumerable: true,
    configurable: true
});
ordinaryDescriptor = Object.getOwnPropertyDescriptor(defaultDescriptors, '289');
check(ordinaryDescriptor.writable && ordinaryDescriptor.enumerable && ordinaryDescriptor.configurable,
      true, 'restored implicit descriptor');
check(defaultDescriptors[9001], 'added-after-materialize', 'value after descriptor collapse');

Object.defineProperty(defaultDescriptors, '323', {
    get: function() { return 'temporary-accessor'; },
    enumerable: true,
    configurable: true
});
check(defaultDescriptors[323], 'temporary-accessor', 'descriptor accessor materialization');
delete defaultDescriptors[323];
check(defaultDescriptors[340], 20, 'neighbor after deleting last descriptor exception');

var frozenIndexes = Object.create(null);
for (var frozenIndex = 0; frozenIndex < 80; frozenIndex++) frozenIndexes[frozenIndex] = frozenIndex;
Object.freeze(frozenIndexes);
var frozenDescriptor = Object.getOwnPropertyDescriptor(frozenIndexes, '40');
check(frozenDescriptor.writable, false, 'freeze materialized writable');
check(frozenDescriptor.enumerable, true, 'freeze retained enumerable');
check(frozenDescriptor.configurable, false, 'freeze materialized configurable');
frozenIndexes[40] = -1;
delete frozenIndexes[41];
check(frozenIndexes[40], 40, 'frozen numeric write rejected');
check(frozenIndexes[41], 41, 'frozen numeric delete rejected');

// Repeated literals exercise both first-time structure construction and the
// cached structure/value-placement path.
function makeMixedLiteral(value) {
    return { tail: value, 9: value + 9, 1: value + 1, head: value + 2 };
}
for (var literalRound = 0; literalRound < 100; literalRound++) {
    var literal = makeMixedLiteral(literalRound);
    check(literal.tail, literalRound, 'cached literal named tail');
    check(literal[9], literalRound + 9, 'cached literal index 9');
    check(literal[1], literalRound + 1, 'cached literal index 1');
    check(literal.head, literalRound + 2, 'cached literal named head');
    check(Reflect.ownKeys(literal).join(','), '1,9,tail,head', 'cached literal key order');
}

// The bulk JSON construction path is used above the transition threshold.
var jsonParts = [];
for (var jsonIndex = 39; jsonIndex >= 0; jsonIndex--) {
    jsonParts.push('"' + jsonIndex + '":' + jsonIndex);
    jsonParts.push('"name' + jsonIndex + '":' + (jsonIndex + 100));
}
var parsedMixed = JSON.parse('{' + jsonParts.join(',') + '}');
for (var jsonIndex = 0; jsonIndex < 40; jsonIndex++) {
    check(parsedMixed[jsonIndex], jsonIndex, 'bulk JSON index value');
    check(parsedMixed['name' + jsonIndex], jsonIndex + 100, 'bulk JSON named value');
}
check(Object.keys(parsedMixed).slice(0, 40).join(','), Array.from({ length: 40 }, function(_, i) { return i; }).join(','), 'bulk JSON numeric order');

// Warm an inline cache before shape/descriptor changes to exercise copying.
function readHot(o) { return o.hot; }
var first = { hot: 42 }, second = { hot: 17 };
for (var i = 0; i < 100; i++) { first['f' + i] = i; second['f' + i] = i; }
for (var i = 0; i < 100; i++) { check(readHot(first), 42, 'warm first'); check(readHot(second), 17, 'warm second'); }
first.extra = 4;
Object.defineProperty(first, 'hot', { value: 53, writable: false });
check(readHot(first), 53, 'changed descriptor');
check(readHot(second), 17, 'other shape intact');
delete first.f20;
check(first.f21, 21, 'copied deletion');
check(second.f20, 20, 'other property intact');
Object.freeze(first);
check(readHot(first), 53, 'frozen lookup');
print('ObjectStructure index tests passed');
