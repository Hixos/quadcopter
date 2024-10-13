use std::collections::HashMap;
use thiserror::Error;

use crate::{
    telemetry::{
        TelemetryDispatcher, TelemetryError, TelemetryReceiver, TelemetrySender, TelemetryService,
    },
    utils::path::Path,
};

#[derive(Debug, Error)]
pub enum Error {
    #[error("Missing configuration for node {0}")]
    MissingConfig(String),

    #[error("Error instantiating node: {0}")]
    NodeInstantiation(Box<dyn std::error::Error + Send + Sync>),
}

pub trait Node {
    fn step(&mut self) -> anyhow::Result<()>;
}

#[derive(Default)]
pub struct NodeManager {
    telemetry: TelemetryService,
    node_configs: HashMap<String, NodeConfig>,
    pub(super) nodes: Vec<Box<dyn Node + Send>>,
}

impl NodeManager {
    pub fn new(telemetry: TelemetryService, node_configs: HashMap<String, NodeConfig>) -> Self {
        NodeManager {
            telemetry,
            node_configs: node_configs,
            nodes: vec![],
        }
    }

    pub fn add_node<
        F: FnOnce(
            NodeContext,
        ) -> Result<Box<dyn Node + Send>, Box<dyn std::error::Error + Send + Sync>>,
    >(
        &mut self,
        name: &str,
        creator: F,
    ) -> Result<(), Error> {
        let config = self
            .node_configs
            .get(name)
            .ok_or(Error::MissingConfig(name.to_string()))?;

        let context = NodeContext::new(NodeTelemetry::new(
            self.telemetry.clone(),
            config.tm_input_map.clone(),
            config.tm_output_map.clone(),
        ));

        self.nodes
            .push(creator(context).map_err(|e| Error::NodeInstantiation(e))?);

        Ok(())
    }
}

#[derive(Debug, Clone, Default)]
pub struct NodeConfig {
    pub tm_input_map: HashMap<String, Path>,
    pub tm_output_map: HashMap<String, Path>,
}

#[derive(Debug)]
pub struct NodeContext {
    tm_dispatcher: NodeTelemetry,
}

impl NodeContext {
    fn new(tm_dispatcher: NodeTelemetry) -> Self {
        Self { tm_dispatcher }
    }

    pub fn telemetry<'a>(&'a self) -> &'a NodeTelemetry {
        &self.tm_dispatcher
    }
}

#[derive(Debug)]
pub struct NodeTelemetry {
    telemetry: TelemetryService,
    input_map: HashMap<String, Path>,
    output_map: HashMap<String, Path>,
}

impl NodeTelemetry {
    pub fn new(
        ts: TelemetryService,
        input_map: HashMap<String, Path>,
        output_map: HashMap<String, Path>,
    ) -> Self {
        NodeTelemetry {
            telemetry: ts,
            input_map,
            output_map,
        }
    }
}

impl TelemetryDispatcher for NodeTelemetry {
    fn publish<T: 'static>(
        &self,
        channel_name: &str,
    ) -> Result<TelemetrySender<T>, TelemetryError> {
        let path = if self.output_map.contains_key(channel_name) {
            self.output_map.get(channel_name).unwrap().clone()
        } else {
            Path::from_str(channel_name).map_err(|_| TelemetryError::InvalidChannelName)?
        };

        self.telemetry.publish::<T>(path.as_str())
    }

    fn subcribe<T: 'static>(
        &self,
        channel_name: &str,
        capacity: usize,
    ) -> Result<TelemetryReceiver<T>, TelemetryError> {
        let path = if self.input_map.contains_key(channel_name) {
            self.input_map.get(channel_name).unwrap().clone()
        } else {
            Path::from_str(channel_name).map_err(|_| TelemetryError::InvalidChannelName)?
        };

        self.telemetry.subcribe::<T>(path.as_str(), capacity)
    }
}

#[cfg(test)]
mod tests {
    use std::sync::mpsc::{channel, Sender};

    use super::*;
    use anyhow::Result;

    #[test]
    fn test_input_remap() -> Result<()> {
        let nt = NodeTelemetry::new(
            TelemetryService::default(),
            HashMap::from([
                ("i1".to_string(), Path::from_str("/a/b/i1")?),
                ("i2".to_string(), Path::from_str("/a/b/i2")?),
            ]),
            HashMap::from([
                ("o1".to_string(), Path::from_str("/a/b/i1")?),
                ("o2".to_string(), Path::from_str("/a/b/i2")?),
            ]),
        );

        let r1 = nt.subcribe::<i32>("i1", 1)?;
        let r1_2 = nt.subcribe::<i32>("/a/b/i1", 1)?;

        assert!(nt.subcribe::<i32>("o2", 1).is_err());
        assert!(nt.subcribe::<i32>("no_remap", 1).is_err());

        let r2 = nt.subcribe::<i32>("i2", 1)?;

        let p1 = nt.publish::<i32>("/a/b/i1")?;
        let p2 = nt.publish::<i32>("/a/b/i2")?;

        p1.send(1);
        assert_eq!(r1.try_recv(), Ok(1));
        assert_eq!(r1_2.try_recv(), Ok(1));

        p2.send(1);
        assert_eq!(r2.try_recv(), Ok(1));
        Ok(())
    }

    #[test]
    fn test_output_remap() -> Result<()> {
        let nt = NodeTelemetry::new(
            TelemetryService::default(),
            HashMap::from([
                ("i1".to_string(), Path::from_str("/a/b/i1")?),
                ("i2".to_string(), Path::from_str("/a/b/i2")?),
            ]),
            HashMap::from([
                ("o1".to_string(), Path::from_str("/a/b/i1")?),
                ("o2".to_string(), Path::from_str("/a/b/i2")?),
            ]),
        );

        let r1 = nt.subcribe::<i32>("/a/b/i1", 1)?;
        let r2 = nt.subcribe::<i32>("/a/b/i2", 1)?;

        assert!(nt.publish::<i32>("i1").is_err());
        assert!(nt.publish::<i32>("no_remap").is_err());

        let p1 = nt.publish::<i32>("o1")?;
        let p2 = nt.publish::<i32>("/a/b/i2")?;

        p1.send(1);
        assert_eq!(r1.try_recv(), Ok(1));

        p2.send(1);
        assert_eq!(r2.try_recv(), Ok(1));
        Ok(())
    }

    struct MockNodeS {
        sender: TelemetrySender<i32>,
        cnt: i32,
    }

    impl MockNodeS {
        fn new(ctx: NodeContext) -> Result<Self> {
            let sender = ctx.telemetry().publish::<i32>("o1")?;
            Ok(MockNodeS { sender, cnt: 0 })
        }
    }
    impl Node for MockNodeS {
        fn step(&mut self) -> Result<()> {
            self.sender.send(self.cnt);
            self.cnt += 1;

            Ok(())
        }
    }

    struct MockNodeR {
        receiver: TelemetryReceiver<i32>,
        cnt: i32,
        feedback: Sender<i32>,
    }

    impl MockNodeR {
        fn new(ctx: NodeContext, feedback: Sender<i32>) -> Result<Self> {
            let receiver = ctx.telemetry().subcribe::<i32>("i1", 1)?;
            Ok(MockNodeR {
                receiver,
                cnt: 0,
                feedback,
            })
        }
    }
    impl Node for MockNodeR {
        fn step(&mut self) -> Result<()> {
            self.cnt = self.receiver.recv().unwrap();
            self.feedback.send(self.cnt).unwrap();

            Ok(())
        }
    }

    #[test]
    fn test_add_node_ok() -> Result<()> {
        let mut nm = NodeManager::new(TelemetryService::default(), HashMap::from([
            (
                "node_s".to_string(),
                NodeConfig {
                    tm_input_map: HashMap::default(),
                    tm_output_map: HashMap::from([("o1".to_string(), Path::from_str("/a/b/c")?)]),
                },
            ),
            (
                "node_r".to_string(),
                NodeConfig {
                    tm_input_map: HashMap::from([("i1".to_string(), Path::from_str("/a/b/c")?)]),
                    tm_output_map: HashMap::default(),
                },
            ),
        ]));

        nm.add_node(
            "node_s",
            |ctx| -> Result<Box<dyn Node + Send>, Box<dyn std::error::Error + Send + Sync>> {
                Ok(Box::new(MockNodeS::new(ctx)?))
            },
        )?;

        let (fb_sender, fb_receiver) = channel();
        nm.add_node(
            "node_r",
            |ctx| -> Result<Box<dyn Node + Send>, Box<dyn std::error::Error + Send + Sync>> {
                Ok(Box::new(MockNodeR::new(ctx, fb_sender)?))
            },
        )?;

        assert_eq!(nm.nodes.len(), 2);

        nm.nodes[0].step().unwrap();
        nm.nodes[1].step().unwrap();

        assert_eq!(fb_receiver.try_recv(), Ok(0));

        nm.nodes[0].step().unwrap();
        nm.nodes[1].step().unwrap();

        assert_eq!(fb_receiver.try_recv(), Ok(1));

        Ok(())
    }
}