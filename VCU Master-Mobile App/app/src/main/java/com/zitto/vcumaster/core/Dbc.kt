package com.zitto.vcumaster.core

import kotlin.math.roundToLong

/**
 * Minimal DBC reader + codec (replaces cantools on the phone).
 * Supports BO_, SG_ (Intel/Motorola, signed/unsigned, simple multiplexing), VAL_, CM_,
 * BA_ "GenMsgCycleTime" and the System*LongSymbol long names. Like cantools, signals are ordered by
 * start bit (stable) and Vector's VECTOR__INDEPENDENT_SIG_MSG is dropped.
 */
class DbcSignal(
    name: String,
    val start: Int,
    val length: Int,
    val littleEndian: Boolean,
    val signed: Boolean,
    val scale: Double,
    val offset: Double,
    val min: Double?,
    val max: Double?,
    val unit: String,
    val receivers: List<String>,
    val muxSwitch: Boolean,
    val muxId: Long?,
) {
    var name: String = name
        internal set
    val choices = LinkedHashMap<Long, String>()
    var comment = ""

    /** cantools utils.start_bit(): sort key for the signal order. */
    val sortBit: Int get() = if (littleEndian) start else 8 * (start / 8) + (7 - start % 8)

    fun extract(data: IntArray): Long {
        var raw = 0L
        if (littleEndian) {
            for (i in 0 until length) {
                val bit = start + i
                val byte = bit / 8
                if (byte < data.size && (data[byte] shr (bit % 8)) and 1 == 1) raw = raw or (1L shl i)
            }
        } else {
            var pos = start
            for (i in 0 until length) {
                val byte = pos / 8
                val b = if (byte < data.size && byte >= 0) (data[byte] shr (pos % 8)) and 1 else 0
                raw = (raw shl 1) or b.toLong()
                pos = if (pos % 8 == 0) pos + 15 else pos - 1
            }
        }
        if (signed && length in 1..63 && raw and (1L shl (length - 1)) != 0L) raw -= (1L shl length)
        return raw
    }

    fun insert(data: IntArray, rawIn: Long) {
        val raw = if (length >= 64) rawIn else rawIn and ((1L shl length) - 1)
        if (littleEndian) {
            for (i in 0 until length) {
                val bit = start + i
                val byte = bit / 8
                if (byte >= data.size) continue
                val m = 1 shl (bit % 8)
                data[byte] = if ((raw shr i) and 1L == 1L) data[byte] or m else data[byte] and m.inv()
            }
        } else {
            var pos = start
            for (i in 0 until length) {
                val byte = pos / 8
                if (byte in data.indices) {
                    val m = 1 shl (pos % 8)
                    val bitVal = (raw shr (length - 1 - i)) and 1L
                    data[byte] = if (bitVal == 1L) data[byte] or m else data[byte] and m.inv()
                }
                pos = if (pos % 8 == 0) pos + 15 else pos - 1
            }
        }
    }

    fun phys(raw: Long): Double = raw * scale + offset

    fun toRaw(v: Double): Long {
        var r = ((v - offset) / scale).roundToLong()
        if (length < 64) {
            val lo = if (signed) -(1L shl (length - 1)) else 0L
            val hi = if (signed) (1L shl (length - 1)) - 1 else (1L shl length) - 1
            r = r.coerceIn(lo, hi)
        }
        return r
    }
}

class DbcMessage(
    val frameId: Long,
    val ext: Boolean,
    name: String,
    val length: Int,
    val senders: List<String>,
) {
    var name: String = name
        internal set
    val signals = mutableListOf<DbcSignal>()
    var cycleMs: Int? = null
    var comment = ""

    /** Physical values of every signal present in this frame (mux-aware). */
    fun decode(data: IntArray): LinkedHashMap<String, Double> {
        val buf = if (data.size >= length) data else data.copyOf(length)
        val out = LinkedHashMap<String, Double>()
        val muxSig = signals.firstOrNull { it.muxSwitch }
        val muxVal = muxSig?.extract(buf)
        for (s in signals) {
            if (s.muxId != null && s.muxId != muxVal) continue
            out[s.name] = s.phys(s.extract(buf))
        }
        return out
    }

    fun encode(values: Map<String, Double>): IntArray {
        val data = IntArray(length)
        for (s in signals) {
            if (s.muxId != null) continue
            s.insert(data, s.toRaw(values[s.name] ?: 0.0))
        }
        return data
    }
}

class DbcDatabase(val messages: List<DbcMessage>) {
    val byId: Map<Long, DbcMessage> = messages.associateBy { it.frameId }
    val byName: Map<String, DbcMessage> = messages.associateBy { it.name }
    val signalCount get() = messages.sumOf { it.signals.size }
}

object DbcParser {
    private val BO_RE = Regex("""^\s*BO_\s+(\d+)\s+(\w+)\s*:\s*(\d+)\s+(\S+)""")
    private val SG_RE = Regex(
        """^\s*SG_\s+(\w+)\s*(M|m\d+M?)?\s*:\s*(\d+)\|(\d+)@([01])([+-])\s*\(\s*([^,]+?)\s*,\s*([^)]+?)\s*\)\s*\[\s*([^|\]]*?)\s*\|\s*([^\]]*?)\s*]\s*"([^"]*)"\s*(.*)$""",
    )
    private val CM_RE = Regex(
        """\bCM_\s+(?:(BO_)\s+(\d+)\s+|(SG_)\s+(\d+)\s+(\w+)\s+|(?:BU_|EV_)\s+\w+\s+)?"([^"]*)"\s*;""",
    )
    private val VAL_RE = Regex("""\bVAL_\s+(\d+)\s+(\w+)\s+""")
    private val VAL_PAIR = Regex("""(-?\d+)\s+"([^"]*)"""")
    private val CYCLE_RE = Regex("""\bBA_\s+"GenMsgCycleTime"\s+BO_\s+(\d+)\s+(\d+)\s*;""")
    private val SIG_LONG_RE = Regex("""\bBA_\s+"SystemSignalLongSymbol"\s+SG_\s+(\d+)\s+(\w+)\s+"([^"]*)"\s*;""")
    private val MSG_LONG_RE = Regex("""\bBA_\s+"SystemMessageLongSymbol"\s+BO_\s+(\d+)\s+"([^"]*)"\s*;""")

    fun parse(text: String): DbcDatabase {
        val msgs = mutableListOf<DbcMessage>()
        val byRaw = HashMap<Long, DbcMessage>()
        var cur: DbcMessage? = null
        for (line in text.lineSequence()) {
            val bo = BO_RE.find(line)
            if (bo != null) {
                val rawId = bo.groupValues[1].toLong()
                if (bo.groupValues[2] == "VECTOR__INDEPENDENT_SIG_MSG") {
                    cur = null            // Vector's container for unplaced signals, not a frame (cantools skips it)
                    continue
                }
                val ext = rawId and 0x80000000L != 0L
                val m = DbcMessage(
                    frameId = rawId and 0x1FFFFFFFL,
                    ext = ext,
                    name = bo.groupValues[2],
                    length = bo.groupValues[3].toInt(),
                    senders = listOf(bo.groupValues[4]).filter { it != "Vector__XXX" },
                )
                msgs += m
                byRaw[rawId] = m
                cur = m
                continue
            }
            val sg = SG_RE.find(line)
            if (sg != null && cur != null) {
                val g = sg.groupValues
                val mux = g[2]
                val lo = g[9].toDoubleOrNull()
                val hi = g[10].toDoubleOrNull()
                val noRange = (lo == null || lo == 0.0) && (hi == null || hi == 0.0)
                cur.signals += DbcSignal(
                    name = g[1],
                    start = g[3].toInt(),
                    length = g[4].toInt(),
                    littleEndian = g[5] == "1",
                    signed = g[6] == "-",
                    scale = g[7].toDoubleOrNull() ?: 1.0,
                    offset = g[8].toDoubleOrNull() ?: 0.0,
                    min = if (noRange) null else lo,
                    max = if (noRange) null else hi,
                    unit = g[11],
                    receivers = g[12].split(Regex("[,\\s]+")).filter { it.isNotBlank() && it != "Vector__XXX" },
                    muxSwitch = mux == "M" || mux.endsWith("M") && mux.startsWith("m"),
                    muxId = if (mux.startsWith("m")) mux.drop(1).removeSuffix("M").toLongOrNull() else null,
                )
                continue
            }
            if (line.isNotBlank() && !line.startsWith(" ") && !line.startsWith("\t")) {
                // any other top-level keyword ends the current message block
                if (!line.trimStart().startsWith("SG_")) cur = null
            }
        }

        for (m in CM_RE.findAll(text)) {
            val g = m.groupValues
            when {
                g[1] == "BO_" -> byRaw[g[2].toLong()]?.comment = g[6]
                g[3] == "SG_" -> byRaw[g[4].toLong()]?.signals?.firstOrNull { it.name == g[5] }?.comment = g[6]
            }
        }
        for (m in VAL_RE.findAll(text)) {
            // value table body: scan to the ';' outside quotes (a repeated-group regex overflows the stack on long tables)
            var i = m.range.last + 1
            var quoted = false
            while (i < text.length && (quoted || text[i] != ';')) {
                if (text[i] == '"') quoted = !quoted
                i++
            }
            val sig = byRaw[m.groupValues[1].toLong()]?.signals?.firstOrNull { it.name == m.groupValues[2] } ?: continue
            for (p in VAL_PAIR.findAll(text.substring(m.range.last + 1, i))) sig.choices[p.groupValues[1].toLong()] = p.groupValues[2]
        }
        for (m in CYCLE_RE.findAll(text)) byRaw[m.groupValues[1].toLong()]?.cycleMs = m.groupValues[2].toIntOrNull()
        for (m in SIG_LONG_RE.findAll(text)) {
            byRaw[m.groupValues[1].toLong()]?.signals?.firstOrNull { it.name == m.groupValues[2] }?.name = m.groupValues[3]
        }
        for (m in MSG_LONG_RE.findAll(text)) byRaw[m.groupValues[1].toLong()]?.name = m.groupValues[2]
        for (m in msgs) {
            val sorted = m.signals.sortedBy { it.sortBit }
            m.signals.clear()
            m.signals.addAll(sorted)
        }

        if (msgs.isEmpty()) throw IllegalArgumentException("no BO_ message definitions found")
        return DbcDatabase(msgs)
    }
}
