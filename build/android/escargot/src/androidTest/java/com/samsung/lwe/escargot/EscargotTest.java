package com.samsung.lwe.escargot;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import androidx.test.ext.junit.runners.AndroidJUnit4;

import org.junit.Test;
import org.junit.runner.RunWith;

import java.util.Optional;

@RunWith(AndroidJUnit4.class)
public class EscargotTest {
    @Test
    public void initTest() {
        Globals.initializeGlobals();
        VMInstance vmInstance = VMInstance.create(Optional.of("en-US"), Optional.of("Asia/Seoul"));
        Context context = Context.create(vmInstance);

        for (int i = 0; i < 300000; i ++) {
            // alloc many trach objects for testing memory management
            JavaScriptValue.create("asdf");
        }

        context = null;
        vmInstance = null;
        Globals.finalizeGlobals();
    }

    @Test
    public void initMultipleTimesTest() {
        // pass if there is no crash
        Globals.initializeGlobals();
        Globals.finalizeGlobals();

        Globals.initializeGlobals();
        Globals.finalizeGlobals();

        Globals.initializeGlobals();
        Globals.finalizeGlobals();
    }

    @Test
    public void muitipleVMTest() {
        Globals.initializeGlobals();
        // pass if there is no crash
        // user can create multiple VM
        VMInstance.create(Optional.empty(), Optional.empty());
        VMInstance.create(Optional.of("en-US"), Optional.empty());
        VMInstance.create(Optional.empty(), Optional.of("Asia/Seoul"));
        Globals.finalizeGlobals();
    }

    @Test
    public void muitipleContextTest() {
        Globals.initializeGlobals();
        // pass if there is no crash
        // user can create multiple context
        VMInstance vm = VMInstance.create(Optional.empty(), Optional.empty());
        Context.create(vm);
        Context.create(vm);
        Context.create(vm);
        Globals.finalizeGlobals();
    }

    @Test
    public void simpleScriptRunTest() {
        Globals.initializeGlobals();

        VMInstance vmInstance = VMInstance.create(Optional.of("en-US"), Optional.of("Asia/Seoul"));
        Context context = Context.create(vmInstance);

        // test script parsing error
        assertFalse(Evaluator.evalScript(context, "@@", "invalid", true).isPresent());
        // test runtime error
        assertFalse(Evaluator.evalScript(context, "throw new Error()", "invalid", true).isPresent());

        assertTrue(Evaluator.evalScript(context, "a = 1", "from_java.js", true).get().toString(context).get().toJavaString().equals("1"));
        assertTrue(Evaluator.evalScript(context, "a", "from_java2.js", true).get().toString(context).get().toJavaString().equals("1"));

        context = null;
        vmInstance = null;
        Globals.finalizeGlobals();
    }

    @Test
    public void attachICUTest() {
        Globals.initializeGlobals();

        VMInstance vmInstance = VMInstance.create(Optional.of("en-US"), Optional.of("Asia/Seoul"));
        Context context = Context.create(vmInstance);

        assertTrue(Evaluator.evalScript(context, "a = new Date('2000/1/1')", "from_java3.js", true).get().toString(context).get().toJavaString().contains("2000"));
        assertTrue(Evaluator.evalScript(context,
                "const koDtf = new Intl.DateTimeFormat(\"ko\", { dateStyle: \"long\" }); koDtf.format(a)", "from_java4.js", true).get().toString(context).get().toJavaString().contains("2000"));

        context = null;
        vmInstance = null;
        Globals.finalizeGlobals();
    }

    @Test
    public void bridgeTest() {
        Globals.initializeGlobals();

        VMInstance vmInstance = VMInstance.create(Optional.of("en-US"), Optional.of("Asia/Seoul"));
        Context context = Context.create(vmInstance);

        class TestBridge extends Bridge.Adapter {
            public boolean called = false;
            @Override
            public Optional<JavaScriptValue> callback(Optional<JavaScriptValue> data) {
                assertFalse(called);
                assertTrue(data.get().isString());
                assertTrue(data.get().asScriptString().toJavaString().equals("dddd"));
                called = true;
                return Optional.of(JavaScriptValue.create(data.get().asScriptString().toJavaString() + "ASdfasdfasdf"));
            }
        };
        TestBridge testBridge = new TestBridge();
        Bridge.register(context, "Native", "addString", testBridge);
        Bridge.register(context, "Native", "returnString", new Bridge.Adapter() {
            @Override
            public Optional<JavaScriptValue> callback(Optional<JavaScriptValue> data) {
                assertFalse(data.isPresent());
                return Optional.of(JavaScriptValue.create("string from java"));
            }
        });
        Bridge.register(context, "Native", "returnNothing", new Bridge.Adapter() {
            @Override
            public Optional<JavaScriptValue> callback(Optional<JavaScriptValue> data) {
                return Optional.empty();
            }
        });

        assertTrue(Evaluator.evalScript(context, "Native.addString('dddd')", "from_java5.js", true).get().asScriptString().toJavaString().equals("ddddASdfasdfasdf"));
        assertTrue(testBridge.called);

        assertTrue(Evaluator.evalScript(context, "Native.returnString()", "from_java6.js", true).get().asScriptString().toJavaString().equals("string from java"));
        assertTrue(Evaluator.evalScript(context, "Native.returnNothing() === undefined", "from_java7.js", true).get().toString(context).get().toJavaString().equals("true"));

        context = null;
        vmInstance = null;
        Globals.finalizeGlobals();
    }

    @Test
    public void nonHeapValueTest() {
        Globals.initializeGlobals();

        JavaScriptValue v = JavaScriptValue.createUndefined();
        assertTrue(v.isUndefined());
        assertFalse(v.isNull());
        assertFalse(v.isNumber());

        v = JavaScriptValue.createNull();
        assertTrue(v.isNull());
        assertFalse(v.isUndefined());
        assertFalse(v.isNumber());

        v = JavaScriptValue.create(123);
        assertTrue(v.isInt32());
        assertEquals(v.asInt32(), 123);
        assertTrue(v.isNumber());
        assertEquals(v.asNumber(), 123.0, 0);

        v = JavaScriptValue.create(true);
        assertTrue(v.isTrue());
        assertTrue(v.isBoolean());
        assertTrue(v.asBoolean());
        assertFalse(v.isInt32());
        assertFalse(v.isUndefined());

        v = JavaScriptValue.create(false);
        assertTrue(v.isFalse());
        assertTrue(v.isBoolean());
        assertFalse(v.asBoolean());
        assertFalse(v.isInt32());
        assertFalse(v.isUndefined());

        Globals.finalizeGlobals();
    }

    @Test
    public void heapValueTest() {
        Globals.initializeGlobals();

        JavaScriptValue v = JavaScriptValue.create(3.14);
        assertTrue(v.isNumber());
        assertEquals(v.asNumber(), 3.14, 0);
        assertFalse(v.isInt32());

        v = JavaScriptValue.create("hello");
        assertTrue(v.isString());
        assertTrue(v.asScriptString().toJavaString().equals("hello"));
        assertFalse(v.isNumber());
        assertFalse(v.isUndefinedOrNull());

        Globals.finalizeGlobals();
    }

    @Test
    public void valueToStringTest() {
        Globals.initializeGlobals();

        VMInstance vmInstance = VMInstance.create(Optional.of("en-US"), Optional.of("Asia/Seoul"));
        Context context = Context.create(vmInstance);

        Optional<JavaScriptString> result = JavaScriptValue.create(123).toString(context);
        assertTrue(result.isPresent());
        assertEquals(result.get().toJavaString(), "123");

        assertTrue(JavaScriptValue.create(Integer.MAX_VALUE).isInt32());
        assertTrue(JavaScriptValue.create(Integer.MAX_VALUE).isNumber());
        result = JavaScriptValue.create(Integer.MAX_VALUE).toString(context);
        assertTrue(result.isPresent());
        assertEquals(result.get().toJavaString(), Integer.MAX_VALUE+"");

        context = null;
        vmInstance = null;
        Globals.finalizeGlobals();
    }

    @Test
    public void valueOperationTest() {
        Globals.initializeGlobals();

        VMInstance vmInstance = VMInstance.create(Optional.of("en-US"), Optional.of("Asia/Seoul"));
        Context context = Context.create(vmInstance);

        assertEquals(JavaScriptValue.create(123).toString(context).get().toJavaString(), "123");
        assertEquals(JavaScriptValue.create(123).toBoolean(context).get(), true);
        assertEquals(JavaScriptValue.create(0).toBoolean(context).get(), false);
        assertEquals(JavaScriptValue.create(123.123).toInteger(context).get(), Double.valueOf(123.0));
        assertEquals(JavaScriptValue.create(123.456).toInt32(context).get(), Integer.valueOf(123));
        assertEquals(JavaScriptValue.create("123").toNumber(context).get(), Double.valueOf(123));
        assertTrue(JavaScriptValue.create("123").toObject(context).get().isObject());
        assertFalse(context.lastThrownException().isPresent());
        assertFalse(JavaScriptValue.createUndefined().toObject(context).isPresent());
        assertTrue(context.lastThrownException().isPresent());

        context = null;
        vmInstance = null;
        Globals.finalizeGlobals();
    }

    @Test
    public void symbolValueTest() {
        Globals.initializeGlobals();

        VMInstance vmInstance = VMInstance.create(Optional.empty(), Optional.empty());
        Context context = Context.create(vmInstance);

        JavaScriptSymbol symbol = JavaScriptValue.create(Optional.empty());
        assertFalse(symbol.description().isPresent());
        assertTrue(symbol.symbolDescriptiveString().toJavaString().equals("Symbol()"));

        symbol = JavaScriptValue.create(Optional.of(JavaScriptString.create("foobar")));
        assertTrue(symbol.description().isPresent());
        assertTrue(symbol.description().get().toJavaString().equals("foobar"));
        assertFalse(symbol.equalsTo(context, JavaScriptValue.create(Optional.of(JavaScriptString.create("foobar")))).get().booleanValue());

        Optional<JavaScriptValue> exception = context.lastThrownException();
        assertFalse(exception.isPresent());
        Optional<JavaScriptString> shouldBeEmpty = symbol.toString(context);
        assertFalse(shouldBeEmpty.isPresent());
        exception = context.lastThrownException();
        assertTrue(exception.isPresent());
        assertEquals(exception.get().toString(context).get().toJavaString(), "TypeError: Cannot convert a Symbol value to a string");

        JavaScriptSymbol symbol1 = JavaScriptSymbol.fromGlobalSymbolRegistry(vmInstance, JavaScriptString.create("foo"));
        JavaScriptSymbol symbol2 = JavaScriptSymbol.fromGlobalSymbolRegistry(vmInstance, JavaScriptString.create("foo"));
        assertTrue(symbol1.equalsTo(context, symbol2).get().booleanValue());

        context = null;
        vmInstance = null;
        Globals.finalizeGlobals();
    }

    @Test
    public void objectCreateReadWriteTest() {
        Globals.initializeGlobals();

        VMInstance vmInstance = VMInstance.create(Optional.empty(), Optional.empty());
        Context context = Context.create(vmInstance);

        JavaScriptObject obj = JavaScriptObject.create(context);
        assertTrue(obj.set(context, JavaScriptValue.create("asdf"), JavaScriptValue.create(123)).get().booleanValue());
        assertTrue(obj.get(context, JavaScriptValue.create("asdf")).get().toNumber(context).get().doubleValue() == 123);

        assertTrue(obj.defineDataProperty(context, JavaScriptValue.create("qwer"), JavaScriptValue.create(123), false, false, false).get().booleanValue());
        assertFalse(obj.defineDataProperty(context, JavaScriptValue.create("qwer"), JavaScriptValue.create(456), false, true, true).get().booleanValue());

        assertTrue(obj.getOwnProperty(context, JavaScriptValue.create("qwer")).get().toNumber(context).get().doubleValue() == 123);

        context = null;
        vmInstance = null;
        Globals.finalizeGlobals();
    }

    @Test
    public void arrayCreateReadWriteTest() {
        Globals.initializeGlobals();

        VMInstance vmInstance = VMInstance.create(Optional.empty(), Optional.empty());
        Context context = Context.create(vmInstance);

        JavaScriptArrayObject arr = JavaScriptArrayObject.create(context);
        assertTrue(arr.set(context, JavaScriptValue.create(3), JavaScriptValue.create(123)).get().booleanValue());
        assertTrue(arr.get(context, JavaScriptValue.create(3)).get().toNumber(context).get().doubleValue() == 123);
        assertTrue(arr.get(context, JavaScriptValue.create("length")).get().toInt32(context).get().intValue() == 4);
        assertTrue(arr.length(context) == 4);

        context = null;
        vmInstance = null;
        Globals.finalizeGlobals();
    }

    @Test
    public void jsonParseStringifyTest() {
        Globals.initializeGlobals();

        VMInstance vmInstance = VMInstance.create(Optional.empty(), Optional.empty());
        Context context = Context.create(vmInstance);

        String testString = "[1, 2, 3]";
        JavaScriptValue result = context.getGlobalObject().jsonParse(context, JavaScriptValue.create(testString)).get();
        assertTrue(result.isArrayObject());
        assertEquals(result.asScriptArrayObject().get(context, JavaScriptValue.create(0)).get().toNumber(context).get().intValue(), 1);
        assertEquals(result.asScriptArrayObject().get(context, JavaScriptValue.create(1)).get().toNumber(context).get().intValue(), 2);
        assertEquals(result.asScriptArrayObject().get(context, JavaScriptValue.create(2)).get().toNumber(context).get().intValue(), 3);
        assertEquals(result.asScriptArrayObject().length(context), 3);

        testString = "{\"a\": \"asdf\"}";
        result = context.getGlobalObject().jsonParse(context, JavaScriptValue.create(testString)).get();
        assertTrue(result.isObject());
        assertEquals(result.asScriptObject().get(context, JavaScriptValue.create("a")).get().asScriptString().toJavaString(), "asdf");
        result.asScriptObject().set(context, JavaScriptValue.create(123), JavaScriptValue.create(456));
        assertEquals(context.getGlobalObject().jsonStringify(context, result).get().toJavaString(), "{\"123\":456,\"a\":\"asdf\"}");

        context = null;
        vmInstance = null;
        Globals.finalizeGlobals();
    }

}