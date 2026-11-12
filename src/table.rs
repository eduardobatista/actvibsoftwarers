//! Minimal column-oriented table with Feather (Arrow IPC) and tab-separated
//! CSV I/O, compatible with the files written by pandas in the Python version.

use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result, anyhow, bail};
use arrow_array::{
    Array, ArrayRef, BooleanArray, Float32Array, Float64Array, Int8Array, Int16Array, Int32Array,
    Int64Array, LargeStringArray, RecordBatch, StringArray, StringViewArray, UInt8Array,
    UInt16Array, UInt32Array, UInt64Array,
};
use arrow_ipc::reader::FileReader;
use arrow_ipc::writer::FileWriter;
use arrow_schema::{DataType, Field, Schema};

#[derive(Clone, Debug, PartialEq)]
pub enum Column {
    F64(Vec<f64>),
    I64(Vec<i64>),
    Bool(Vec<bool>),
    Str(Vec<Option<String>>),
}

impl Column {
    pub fn len(&self) -> usize {
        match self {
            Column::F64(v) => v.len(),
            Column::I64(v) => v.len(),
            Column::Bool(v) => v.len(),
            Column::Str(v) => v.len(),
        }
    }

    /// Numeric view of the column (strings become NaN).
    pub fn as_f64(&self) -> Vec<f64> {
        match self {
            Column::F64(v) => v.clone(),
            Column::I64(v) => v.iter().map(|x| *x as f64).collect(),
            Column::Bool(v) => v.iter().map(|x| if *x { 1.0 } else { 0.0 }).collect(),
            Column::Str(v) => v
                .iter()
                .map(|s| {
                    s.as_deref()
                        .and_then(|s| s.parse().ok())
                        .unwrap_or(f64::NAN)
                })
                .collect(),
        }
    }

    pub fn cell_string(&self, row: usize) -> String {
        match self {
            Column::F64(v) => fmt_f64(v[row]),
            Column::I64(v) => v[row].to_string(),
            Column::Bool(v) => {
                if v[row] {
                    "True".into()
                } else {
                    "False".into()
                }
            }
            Column::Str(v) => v[row].clone().unwrap_or_default(),
        }
    }

    fn take_rows(&self, keep: &[bool]) -> Column {
        fn f<T: Clone>(v: &[T], keep: &[bool]) -> Vec<T> {
            v.iter()
                .zip(keep)
                .filter(|(_, k)| **k)
                .map(|(x, _)| x.clone())
                .collect()
        }
        match self {
            Column::F64(v) => Column::F64(f(v, keep)),
            Column::I64(v) => Column::I64(f(v, keep)),
            Column::Bool(v) => Column::Bool(f(v, keep)),
            Column::Str(v) => Column::Str(f(v, keep)),
        }
    }
}

/// Formats floats like Python's `repr` for the common cases.
pub fn fmt_f64(v: f64) -> String {
    if v.is_nan() {
        return String::new();
    }
    if v.is_infinite() {
        return if v > 0.0 { "inf".into() } else { "-inf".into() };
    }
    if v == v.trunc() && v.abs() < 1e16 {
        return format!("{v:.1}");
    }
    let a = v.abs();
    if !(1e-4..1e16).contains(&a) {
        // Python style exponent: 1e-05
        let s = format!("{v:e}");
        if let Some((m, e)) = s.split_once('e') {
            let (sign, digits) = if let Some(d) = e.strip_prefix('-') {
                ("-", d)
            } else {
                ("+", e)
            };
            return format!("{m}e{sign}{digits:0>2}");
        }
        return s;
    }
    format!("{v}")
}

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Table {
    pub names: Vec<String>,
    pub columns: Vec<Column>,
}

impl Table {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, name: impl Into<String>, col: Column) {
        self.names.push(name.into());
        self.columns.push(col);
    }

    pub fn nrows(&self) -> usize {
        self.columns.first().map(|c| c.len()).unwrap_or(0)
    }

    pub fn get(&self, name: &str) -> Option<&Column> {
        self.names
            .iter()
            .position(|n| n == name)
            .map(|i| &self.columns[i])
    }

    pub fn has(&self, name: &str) -> bool {
        self.get(name).is_some()
    }

    pub fn f64_col(&self, name: &str) -> Result<Vec<f64>> {
        self.get(name)
            .map(|c| c.as_f64())
            .ok_or_else(|| anyhow!("Column '{name}' not found."))
    }

    pub fn drop_rows(&mut self, rows: &[usize]) {
        let n = self.nrows();
        let keep: Vec<bool> = (0..n).map(|r| !rows.contains(&r)).collect();
        for c in self.columns.iter_mut() {
            *c = c.take_rows(&keep);
        }
    }

    /// Appends a row given as (name, column-with-one-value). Missing columns
    /// are filled with nulls/NaN; new columns are created.
    pub fn append_row(&mut self, row: Vec<(&str, Column)>) -> Result<()> {
        let n = self.nrows();
        for (name, val) in row.iter() {
            if !self.has(name) {
                let filler = match val {
                    Column::F64(_) => Column::F64(vec![f64::NAN; n]),
                    Column::I64(_) => Column::I64(vec![0; n]),
                    Column::Bool(_) => Column::Bool(vec![false; n]),
                    Column::Str(_) => Column::Str(vec![None; n]),
                };
                self.push(*name, filler);
            }
        }
        for (i, name) in self.names.clone().iter().enumerate() {
            let v = row.iter().find(|(n, _)| n == name).map(|(_, c)| c);
            match (&mut self.columns[i], v) {
                (Column::F64(c), Some(Column::F64(x))) => c.push(x[0]),
                (Column::F64(c), Some(Column::I64(x))) => c.push(x[0] as f64),
                (Column::I64(c), Some(Column::I64(x))) => c.push(x[0]),
                (Column::I64(c), Some(Column::F64(x))) => c.push(x[0] as i64),
                (Column::Bool(c), Some(Column::Bool(x))) => c.push(x[0]),
                (Column::Str(c), Some(Column::Str(x))) => c.push(x[0].clone()),
                (Column::Str(c), Some(other)) => c.push(Some(other.cell_string(0))),
                (Column::F64(c), None) => c.push(f64::NAN),
                (Column::I64(c), None) => c.push(0),
                (Column::Bool(c), None) => c.push(false),
                (Column::Str(c), None) => c.push(None),
                (_, Some(_)) => bail!("Type mismatch for column '{name}'."),
            }
        }
        Ok(())
    }

    fn to_record_batch(&self) -> Result<RecordBatch> {
        let mut fields = Vec::new();
        let mut arrays: Vec<ArrayRef> = Vec::new();
        for (name, col) in self.names.iter().zip(&self.columns) {
            let (dt, arr): (DataType, ArrayRef) = match col {
                Column::F64(v) => (DataType::Float64, Arc::new(Float64Array::from(v.clone()))),
                Column::I64(v) => (DataType::Int64, Arc::new(Int64Array::from(v.clone()))),
                Column::Bool(v) => (DataType::Boolean, Arc::new(BooleanArray::from(v.clone()))),
                Column::Str(v) => (DataType::Utf8, Arc::new(StringArray::from(v.clone()))),
            };
            fields.push(Field::new(name, dt, true));
            arrays.push(arr);
        }
        Ok(RecordBatch::try_new(Arc::new(Schema::new(fields)), arrays)?)
    }

    pub fn write_feather(&self, path: &Path) -> Result<()> {
        let batch = self.to_record_batch()?;
        let file =
            File::create(path).with_context(|| format!("Cannot create {}", path.display()))?;
        let mut w = FileWriter::try_new(BufWriter::new(file), &batch.schema())?;
        w.write(&batch)?;
        w.finish()?;
        Ok(())
    }

    pub fn read_feather(path: &Path) -> Result<Table> {
        let file = File::open(path).with_context(|| format!("Cannot open {}", path.display()))?;
        let reader =
            FileReader::try_new(file, None).context("Not a valid Feather (Arrow IPC) file")?;
        let schema = reader.schema();
        let mut table = Table::new();
        for f in schema.fields() {
            let col = match f.data_type() {
                DataType::Float64
                | DataType::Float32
                | DataType::Int8
                | DataType::Int16
                | DataType::Int32
                | DataType::UInt8
                | DataType::UInt16
                | DataType::UInt32
                | DataType::UInt64 => Column::F64(vec![]),
                DataType::Int64 => Column::I64(vec![]),
                DataType::Boolean => Column::Bool(vec![]),
                _ => Column::Str(vec![]),
            };
            table.push(f.name().clone(), col);
        }
        for batch in reader {
            let batch = batch?;
            for (i, col) in table.columns.iter_mut().enumerate() {
                append_array(col, batch.column(i).as_ref())?;
            }
        }
        // pandas stores a RangeIndex in metadata only; drop a serialized "__index_level_0__".
        if let Some(pos) = table.names.iter().position(|n| n == "__index_level_0__") {
            table.names.remove(pos);
            table.columns.remove(pos);
        }
        Ok(table)
    }

    /// Writes a tab-separated file like `DataFrame.to_csv(sep="\t")` (with index column).
    pub fn write_csv(&self, path: &Path) -> Result<()> {
        let file =
            File::create(path).with_context(|| format!("Cannot create {}", path.display()))?;
        let mut w = BufWriter::new(file);
        let quote = |s: &str| -> String {
            if s.contains(['\t', '"', '\n', '\r']) {
                format!("\"{}\"", s.replace('"', "\"\""))
            } else {
                s.to_string()
            }
        };
        let header: Vec<String> = self.names.iter().map(|n| quote(n)).collect();
        writeln!(w, "\t{}", header.join("\t"))?;
        for r in 0..self.nrows() {
            write!(w, "{r}")?;
            for c in &self.columns {
                write!(w, "\t{}", quote(&c.cell_string(r)))?;
            }
            writeln!(w)?;
        }
        w.flush()?;
        Ok(())
    }
}

fn append_array(col: &mut Column, arr: &dyn Array) -> Result<()> {
    let any = arr.as_any();
    macro_rules! num {
        ($t:ty, $v:expr) => {
            if let Some(a) = any.downcast_ref::<$t>() {
                $v.extend((0..a.len()).map(|i| {
                    if a.is_null(i) {
                        f64::NAN
                    } else {
                        a.value(i) as f64
                    }
                }));
                return Ok(());
            }
        };
    }
    match col {
        Column::F64(v) => {
            num!(Float64Array, v);
            num!(Float32Array, v);
            num!(Int8Array, v);
            num!(Int16Array, v);
            num!(Int32Array, v);
            num!(UInt8Array, v);
            num!(UInt16Array, v);
            num!(UInt32Array, v);
            num!(UInt64Array, v);
            bail!("Unsupported numeric column type {:?}", arr.data_type())
        }
        Column::I64(v) => {
            let a = any
                .downcast_ref::<Int64Array>()
                .ok_or_else(|| anyhow!("Expected Int64 column"))?;
            v.extend((0..a.len()).map(|i| if a.is_null(i) { 0 } else { a.value(i) }));
            Ok(())
        }
        Column::Bool(v) => {
            let a = any
                .downcast_ref::<BooleanArray>()
                .ok_or_else(|| anyhow!("Expected Boolean column"))?;
            v.extend((0..a.len()).map(|i| !a.is_null(i) && a.value(i)));
            Ok(())
        }
        Column::Str(v) => {
            if let Some(a) = any.downcast_ref::<StringArray>() {
                v.extend((0..a.len()).map(|i| (!a.is_null(i)).then(|| a.value(i).to_string())));
            } else if let Some(a) = any.downcast_ref::<LargeStringArray>() {
                v.extend((0..a.len()).map(|i| (!a.is_null(i)).then(|| a.value(i).to_string())));
            } else if let Some(a) = any.downcast_ref::<StringViewArray>() {
                v.extend((0..a.len()).map(|i| (!a.is_null(i)).then(|| a.value(i).to_string())));
            } else if arr.data_type() == &DataType::Null {
                v.extend(std::iter::repeat_n(None, arr.len()));
            } else {
                // Fallback: render via arrow's display.
                let fmt = arrow_cast_display(arr);
                v.extend(fmt);
            }
            Ok(())
        }
    }
}

fn arrow_cast_display(arr: &dyn Array) -> Vec<Option<String>> {
    (0..arr.len())
        .map(|i| {
            if arr.is_null(i) {
                None
            } else {
                Some(format!("{:?}", arr.slice(i, 1)))
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn feather_roundtrip() {
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("t.feather");
        let mut t = Table::new();
        t.push("time", Column::F64(vec![0.0, 0.004, 0.008]));
        t.push("n", Column::I64(vec![1, 2, 3]));
        t.push(
            "log",
            Column::Str(vec![Some(" ".into()), None, Some("Stopped".into())]),
        );
        t.write_feather(&p).unwrap();
        let r = Table::read_feather(&p).unwrap();
        assert_eq!(r, t);
    }

    #[test]
    fn csv_like_pandas() {
        let dir = tempfile::tempdir().unwrap();
        let p = dir.path().join("t.csv");
        let mut t = Table::new();
        t.push("time", Column::F64(vec![0.0, 0.004]));
        t.push("log", Column::Str(vec![Some("Started".into()), None]));
        t.write_csv(&p).unwrap();
        let s = std::fs::read_to_string(&p).unwrap();
        assert_eq!(s, "\ttime\tlog\n0\t0.0\tStarted\n1\t0.004\t\n");
    }

    #[test]
    fn python_like_float_format() {
        assert_eq!(fmt_f64(1e-5), "1e-05");
        assert_eq!(fmt_f64(0.25), "0.25");
        assert_eq!(fmt_f64(3.0), "3.0");
    }

    #[test]
    fn append_and_drop_rows() {
        let mut t = Table::new();
        t.append_row(vec![
            ("filename", Column::Str(vec![Some("a".into())])),
            ("mu", Column::F64(vec![0.1])),
        ])
        .unwrap();
        t.append_row(vec![
            ("filename", Column::Str(vec![Some("b".into())])),
            ("mu", Column::F64(vec![0.2])),
        ])
        .unwrap();
        assert_eq!(t.nrows(), 2);
        t.drop_rows(&[0]);
        assert_eq!(t.get("filename").unwrap().cell_string(0), "b");
    }
}

#[cfg(test)]
mod interop {
    use super::*;

    /// Writes sample files for cross-checking with pandas, and reads any
    /// pandas-written `from_pandas.feather` found in `ACTVIB_INTEROP_DIR`.
    #[test]
    #[ignore]
    fn pandas_interop() {
        let dir = std::path::PathBuf::from(
            std::env::var("ACTVIB_INTEROP_DIR").expect("set ACTVIB_INTEROP_DIR"),
        );
        let mut t = Table::new();
        t.push("time", Column::F64(vec![0.0, 0.004, 0.008]));
        t.push("perturb", Column::F64(vec![0.1, -0.2, 0.3]));
        t.push("memctrl", Column::I64(vec![100, 200, 300]));
        t.push(
            "log",
            Column::Str(vec![Some(" ".into()), None, Some("Stopped".into())]),
        );
        t.write_feather(&dir.join("from_rust.feather")).unwrap();
        t.write_csv(&dir.join("from_rust.csv")).unwrap();
        let p = dir.join("from_pandas.feather");
        if p.exists() {
            let r = Table::read_feather(&p).unwrap();
            println!("names={:?}", r.names);
            for (n, c) in r.names.iter().zip(&r.columns) {
                println!(
                    "{n}: {}",
                    (0..r.nrows())
                        .map(|i| c.cell_string(i))
                        .collect::<Vec<_>>()
                        .join(", ")
                );
            }
        }
    }
}
