/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package lcmtest;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class byte_array_t implements lcm.lcm.LCMEncodable
{
    public int num_bytes;
    public byte data[];
 
    public byte_array_t()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x870c7477e270debfL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(lcmtest.byte_array_t.class))
            return 0L;
 
        classes.add(lcmtest.byte_array_t.class);
        long hash = LCM_FINGERPRINT_BASE
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
        outs.writeInt(this.num_bytes); 
 
        if (this.num_bytes > 0)
            outs.write(this.data, 0, (int) num_bytes);
 
    }
 
    public byte_array_t(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public byte_array_t(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static lcmtest.byte_array_t _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        lcmtest.byte_array_t o = new lcmtest.byte_array_t();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.num_bytes = ins.readInt();
 
        this.data = new byte[(int) num_bytes];
        ins.readFully(this.data, 0, (int) num_bytes); 
    }
 
    public lcmtest.byte_array_t copy()
    {
        lcmtest.byte_array_t outobj = new lcmtest.byte_array_t();
        outobj.num_bytes = this.num_bytes;
 
        outobj.data = new byte[(int) num_bytes];
        if (this.num_bytes > 0)
            System.arraycopy(this.data, 0, outobj.data, 0, (int) this.num_bytes); 
        return outobj;
    }
 
}
