/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package lcmtest2;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class cross_package_t implements lcm.lcm.LCMEncodable
{
    public lcmtest.primitives_t primitives;
    public lcmtest2.another_type_t another;
 
    public cross_package_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xbbd3dd8a23ec1955L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtest2.cross_package_t.class))
            return 0L;
 
        classes.add(lcmtest2.cross_package_t.class);
        long hash = LCM_FINGERPRINT_BASE
             + lcmtest.primitives_t._hashRecursive(classes)
             + lcmtest2.another_type_t._hashRecursive(classes)
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        this.primitives._encodeRecursive(outs); 
 
        this.another._encodeRecursive(outs); 
 
    }
 
    public cross_package_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public cross_package_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtest2.cross_package_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtest2.cross_package_t o = new lcmtest2.cross_package_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.primitives = lcmtest.primitives_t._decodeRecursiveFactory(ins);
 
        this.another = lcmtest2.another_type_t._decodeRecursiveFactory(ins);
 
    }
 
    public lcmtest2.cross_package_t copy()
    {
        lcmtest2.cross_package_t outobj = new lcmtest2.cross_package_t();
        outobj.primitives = this.primitives.copy();
 
        outobj.another = this.another.copy();
 
        return outobj;
    }
 
}
