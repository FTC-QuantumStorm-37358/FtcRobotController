package org.firstinspires.ftc.teamcode.biobuzz.desktop;

import java.util.*;

/** Small dependency-free JSON codec for the local bridge; excluded from the robot build. */
final class Json {
    private final String text;private int at,depth;
    private Json(String text){this.text=text;}
    static Object parse(String text){Json p=new Json(text);Object v=p.value();p.space();if(p.at!=text.length())throw new IllegalArgumentException("Trailing JSON");return v;}
    private void space(){while(at<text.length()&&Character.isWhitespace(text.charAt(at)))at++;}
    private char take(){if(at>=text.length())throw new IllegalArgumentException("Incomplete JSON");return text.charAt(at++);}
    private void expect(char c){space();if(take()!=c)throw new IllegalArgumentException("Invalid JSON");}
    private Object value(){
        space();if(++depth>24)throw new IllegalArgumentException("JSON nesting too deep");Object result;char c=at<text.length()?text.charAt(at):'\0';
        if(c=='{'){at++;Map<String,Object> m=new LinkedHashMap<String,Object>();space();if(at<text.length()&&text.charAt(at)=='}')at++;else while(true){space();String key=string();expect(':');if(m.containsKey(key))throw new IllegalArgumentException("Duplicate JSON key");m.put(key,value());space();char end=take();if(end=='}')break;if(end!=',')throw new IllegalArgumentException("Invalid object");}result=m;}
        else if(c=='['){at++;List<Object> list=new ArrayList<Object>();space();if(at<text.length()&&text.charAt(at)==']')at++;else while(true){list.add(value());space();char end=take();if(end==']')break;if(end!=',')throw new IllegalArgumentException("Invalid array");}result=list;}
        else if(c=='"')result=string();
        else if(text.startsWith("true",at)){at+=4;result=Boolean.TRUE;}
        else if(text.startsWith("false",at)){at+=5;result=Boolean.FALSE;}
        else if(text.startsWith("null",at)){at+=4;result=null;}
        else{int begin=at;while(at<text.length()&&"-+0123456789.eE".indexOf(text.charAt(at))>=0)at++;String n=text.substring(begin,at);if(!n.matches("-?(0|[1-9][0-9]*)(\\.[0-9]+)?([eE][+-]?[0-9]+)?"))throw new IllegalArgumentException("Invalid number");double number=Double.parseDouble(n);if(!Double.isFinite(number))throw new IllegalArgumentException("Nonfinite number");result=number;}
        depth--;return result;
    }
    private String string(){
        if(take()!='"')throw new IllegalArgumentException("Expected string");StringBuilder s=new StringBuilder();
        while(true){char c=take();if(c=='"')return s.toString();if(c<' ')throw new IllegalArgumentException("Control character");
            if(c=='\\'){char e=take();switch(e){case '"':case '\\':case '/':c=e;break;case 'b':c='\b';break;case 'f':c='\f';break;case 'n':c='\n';break;case 'r':c='\r';break;case 't':c='\t';break;case 'u':if(at+4>text.length())throw new IllegalArgumentException("Unicode escape");c=(char)Integer.parseInt(text.substring(at,at+4),16);at+=4;break;default:throw new IllegalArgumentException("Invalid escape");}}s.append(c);}
    }
    static String write(Object value){
        if(value==null)return "null";
        if(value instanceof String){StringBuilder s=new StringBuilder("\"");for(char c:((String)value).toCharArray()){if(c=='"'||c=='\\')s.append('\\').append(c);else if(c<' ')s.append(String.format(Locale.ROOT,"\\u%04x",(int)c));else s.append(c);}return s.append('"').toString();}
        if(value instanceof Number){if(!Double.isFinite(((Number)value).doubleValue()))throw new IllegalArgumentException("Nonfinite output");return value.toString();}
        if(value instanceof Boolean)return value.toString();
        if(value instanceof Map){StringJoiner j=new StringJoiner(",","{","}");for(Map.Entry<?,?> e:((Map<?,?>)value).entrySet())j.add(write(e.getKey().toString())+":"+write(e.getValue()));return j.toString();}
        if(value instanceof Iterable){StringJoiner j=new StringJoiner(",","[","]");for(Object e:(Iterable<?>)value)j.add(write(e));return j.toString();}
        throw new IllegalArgumentException("Unsupported output type");
    }
    static Map<String,Object> object(Object value){if(!(value instanceof Map))throw new IllegalArgumentException("Expected object");@SuppressWarnings("unchecked")Map<String,Object> m=(Map<String,Object>)value;return m;}
    static double number(Map<String,Object> m,String key){Object n=m.get(key);if(!(n instanceof Number))throw new IllegalArgumentException("Missing number "+key);return ((Number)n).doubleValue();}
    static int integer(Map<String,Object> m,String key){double n=number(m,key);if(n!=Math.rint(n)||n<Integer.MIN_VALUE||n>Integer.MAX_VALUE)throw new IllegalArgumentException("Expected integer "+key);return (int)n;}
    static boolean bool(Map<String,Object> m,String key){Object b=m.get(key);if(!(b instanceof Boolean))throw new IllegalArgumentException("Missing boolean "+key);return (Boolean)b;}
    static String string(Map<String,Object> m,String key){Object s=m.get(key);if(!(s instanceof String))throw new IllegalArgumentException("Missing string "+key);return (String)s;}
    static Map<String,Object> map(Object... values){Map<String,Object> m=new LinkedHashMap<String,Object>();for(int i=0;i<values.length;i+=2)m.put((String)values[i],values[i+1]);return m;}
}
