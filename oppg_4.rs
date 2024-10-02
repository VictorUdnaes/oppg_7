
use std::fs;
use std::io::BufRead;

fn main() {
    // Specify the file path
    let file_path = "navn.txt";
    let mut hash_table = HashTable::new(139);

    // Read the entire file contents into a String
    let contents = fs::read_to_string(file_path).expect("Failed to read file");
    let lines = contents.lines();

    // Insert each line into the hash table
    for line in lines {
        hash_table.put(line.to_string());
    }

    println!("{} was found!", hash_table.get(String::from("Vitaliy Konstantinovich Bravikov")));
    let collision_rate: f32 = (hash_table.collisions as f32 / 124.0f32) * 100.0;
    println!("Collision rate: {}", collision_rate); // Print total collisions
}

#[derive(Clone)]
pub struct HashNode {
    pub data: String,
    pub next: Option<Box<HashNode>>
}

impl HashNode {
    fn new(data:String, next:Option<Box<HashNode>>) -> HashNode {
        HashNode {data, next}
    }
}

pub struct HashTable {
    pub table: Vec<Option<HashNode>>,
    pub size: usize,
    pub collisions: usize,
}

impl HashTable {
    fn new(size: usize) -> HashTable {
        let table = vec![None; size];
        HashTable { table, size, collisions: 0 }
    }

    fn put(&mut self, s: String) {
        let location = Self::compute_hash(&s, self.size);
        match self.table.get_mut(location) {
            Some(Some(value)) => {
                let mut current_node = value;
                while let Some(ref mut next_node) = current_node.next {
                    println!("{} --> {} during put",current_node.data, next_node.data);
                    current_node = next_node;
                    self.collisions += 1;
                }
                self.collisions += 1;
                let node = HashNode::new(s, None);
                current_node.next = Some(Box::new(node));
            }
            Some(None) => {
                let node = HashNode::new(s, None);
                self.table[location] = Some(node);
            }
            None => {
                let node = HashNode::new(s, None);
                self.table.insert(location, Some(node));
            }
        }
    }

    fn get(&self, s: String) -> String {
        let location = Self::compute_hash(&s, self.size);

        let mut current_node = match self.table.get(location) {
            Some(Some(value)) => value,
            _ => return String::from("No value found"),
        };

        if current_node.data == s {
            current_node.data.clone()
        } else {
            while let Some(ref next_node) = current_node.next {
                if next_node.data == s {
                    return next_node.data.clone();
                }
                println!("{} --X {} during get",current_node.data, next_node.data);
                current_node = next_node;
            }
            current_node.data.clone()
        }
    }

    fn compute_hash(s: &String, m: usize) -> usize {
        let mut hash = 0;
        for c in s.chars() {
            hash = (hash * 7 + c as usize) % m;
        }
        hash
    }
}

// Oppgave 2--------------------------------------------------------------------------------------------------------------------------
use std::time::Instant;
use rand::Rng;

const N: usize = 12999959;

fn main() {
    let arr = generate_random_array(); // Allocate array on the heap using Vec
    let mut hash_table = HashTable::new(vec![0; N]);

    let start = Instant::now();
    for i in 0..10000000 {
        hash_table.put(arr[i]);
    }
    println!("time elapsed: {}ms", start.elapsed().as_millis());
    println!("Load factor: {}", hash_table.load_factor());
}

pub struct HashTable {
    pub table: Vec<usize>,
    pub size: usize,
    pub collisions: usize,
}

impl HashTable {
    fn new(table: Vec<usize>) -> HashTable {
        HashTable { table, size: N, collisions: 0 }
    }

    fn put(&mut self, k: usize) {
        let mut location = Self::h1(&self, k);
        if self.table[location] != 0 {
            let step_size = Self::h2(&self, k);
            self.collisions += 1;
            while self.table[location] != 0 {
                location = (location + step_size) % self.size;
            }
        }
        self.table[location] = k;
    }

    fn get(&self, k: usize) -> usize {
        let mut location = Self::h1(&self, k);
        if self.table[location] != 0 {
            self.table[location]
        } else {
            loop {
                location = (Self::h2(&self, k) + location) % self.size;
                if self.table[location] != 0 {
                    return self.table[location]
                }
            }
        }
    }

    fn h1(&self, key: usize) -> usize {
        (key) % self.size
    }

    fn h2(&self, key: usize) -> usize {
        1 + (key) % (self.size - 1)
    }

    fn load_factor(&self) -> f64 {
        println!("Collisions: {}", self.collisions);
        self.collisions as f64 / self.size as f64
    }
}

fn generate_random_array() -> Vec<usize> {
    let mut rng = rand::thread_rng();
    let mut arr = vec![0; N];
    for i in 0..10000000 {
        arr[i] = rng.gen_range(0..1_000_000_000);
    }
    arr
}
