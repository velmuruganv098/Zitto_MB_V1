package com.zitto.vcumaster.core

import org.json.JSONObject

/**
 * The DBC library shipped with VCU Master (same tree as CAN_DBC_Simulator's
 * dbc_library/<Product>/<Vendor>/ *.dbc) and automatic DBC matching. Port of library.py.
 *
 * The bridge forwards every CAN frame, but frames are only decoded once a DBC that contains their
 * identifier is loaded. The frame identifiers of every library DBC are indexed (assets/library_index.json,
 * generated with cantools from the desktop tool's dbc_library/) so the frames seen on the bus can be
 * matched to the DBC that describes them (e.g. a Daly BMS on 0x18904001..0x18984001).
 */
class LibItem(val id: String, val product: String, val vendor: String, val file: String, val ids: Set<Pair<Long, Boolean>>) {
    val messages get() = ids.size
}

data class LibCandidate(
    val id: String, val product: String, val vendor: String, val file: String,
    val matched: Int, val messages: Int, val coverage: Double, val explains: Double,
    val ids: List<String>, val buses: List<Int> = emptyList(),
)

data class LibMatch(val unknown: Int, val candidates: List<LibCandidate>, val unknownIds: List<String>)

class DbcLibrary(indexJson: String) {
    val items: List<LibItem>

    init {
        val out = ArrayList<LibItem>()
        val j = JSONObject(indexJson)
        for (rel in j.keys().asSequence().sorted()) {
            val arr = j.getJSONArray(rel)
            val ids = HashSet<Pair<Long, Boolean>>()
            for (i in 0 until arr.length()) {
                val e = arr.getJSONArray(i)
                ids += e.getLong(0) to e.getBoolean(1)
            }
            val parts = rel.split("/")
            out += LibItem(
                id = rel,
                product = if (parts.size > 1) parts[0] else "",
                vendor = if (parts.size > 2) parts[1] else "",
                file = parts.last(),
                ids = ids,
            )
        }
        items = out
    }

    val byId: Map<String, LibItem> = items.associateBy { it.id }

    fun has(id: String) = id in byId

    /** Library DBCs ranked by how many of the seen identifiers they define. */
    fun match(seen: Set<Pair<Long, Boolean>>, exclude: Set<String> = emptySet()): List<LibCandidate> {
        if (seen.isEmpty()) return emptyList()
        val ranked = ArrayList<LibCandidate>()
        for (i in items) {
            if (i.id in exclude || i.ids.isEmpty()) continue
            val hit = seen.filter { it in i.ids }
            if (hit.isEmpty()) continue
            ranked += LibCandidate(
                id = i.id, product = i.product, vendor = i.vendor, file = i.file,
                matched = hit.size, messages = i.ids.size,
                coverage = Math.round(1000.0 * hit.size / i.ids.size) / 1000.0,
                explains = Math.round(1000.0 * hit.size / seen.size) / 1000.0,
                ids = hit.map { idText(it) }.sorted().take(12),
            )
        }
        ranked.sortWith(compareBy<LibCandidate>({ -it.matched }, { -it.coverage }, { it.messages }))
        return ranked.take(10)
    }

    companion object {
        fun idText(k: Pair<Long, Boolean>) = "0x" + k.first.toString(16).uppercase() + if (k.second) " EXT" else ""
    }
}
