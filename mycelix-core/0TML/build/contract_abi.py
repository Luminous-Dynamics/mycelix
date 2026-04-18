"""Generated ABI for ZeroTrustMLGradientStorage contract"""

CONTRACT_ABI = [
  {
    "type": "constructor",
    "inputs": [],
    "stateMutability": "nonpayable"
  },
  {
    "type": "function",
    "name": "storeGradient",
    "inputs": [
      {
        "name": "gradientId",
        "type": "string"
      },
      {
        "name": "nodeIdHash",
        "type": "bytes32"
      },
      {
        "name": "roundNum",
        "type": "uint256"
      },
      {
        "name": "gradientHash",
        "type": "string"
      },
      {
        "name": "pogqScore",
        "type": "uint256"
      },
      {
        "name": "zkpocVerified",
        "type": "bool"
      }
    ],
    "outputs": [],
    "stateMutability": "nonpayable"
  },
  {
    "type": "function",
    "name": "getGradient",
    "inputs": [
      {
        "name": "gradientId",
        "type": "string"
      }
    ],
    "outputs": [
      {
        "type": "string"
      },
      {
        "type": "bytes32"
      },
      {
        "type": "uint256"
      },
      {
        "type": "string"
      },
      {
        "type": "uint256"
      },
      {
        "type": "bool"
      },
      {
        "type": "uint256"
      },
      {
        "type": "address"
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "function",
    "name": "getGradientsByRound",
    "inputs": [
      {
        "name": "roundNum",
        "type": "uint256"
      }
    ],
    "outputs": [
      {
        "type": "string[]"
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "function",
    "name": "gradientExists",
    "inputs": [
      {
        "name": "gradientId",
        "type": "string"
      }
    ],
    "outputs": [
      {
        "type": "bool"
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "function",
    "name": "issueCredit",
    "inputs": [
      {
        "name": "holderHash",
        "type": "bytes32"
      },
      {
        "name": "amount",
        "type": "uint256"
      },
      {
        "name": "earnedFrom",
        "type": "string"
      }
    ],
    "outputs": [],
    "stateMutability": "nonpayable"
  },
  {
    "type": "function",
    "name": "getCreditBalance",
    "inputs": [
      {
        "name": "holderHash",
        "type": "bytes32"
      }
    ],
    "outputs": [
      {
        "type": "uint256"
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "function",
    "name": "getCreditHistory",
    "inputs": [
      {
        "name": "holderHash",
        "type": "bytes32"
      }
    ],
    "outputs": [
      {
        "type": "tuple[]",
        "components": [
          {
            "name": "holderHash",
            "type": "bytes32"
          },
          {
            "name": "amount",
            "type": "uint256"
          },
          {
            "name": "earnedFrom",
            "type": "string"
          },
          {
            "name": "timestamp",
            "type": "uint256"
          },
          {
            "name": "creditId",
            "type": "uint256"
          }
        ]
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "function",
    "name": "getCreditCount",
    "inputs": [
      {
        "name": "holderHash",
        "type": "bytes32"
      }
    ],
    "outputs": [
      {
        "type": "uint256"
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "function",
    "name": "logByzantineEvent",
    "inputs": [
      {
        "name": "nodeIdHash",
        "type": "bytes32"
      },
      {
        "name": "roundNum",
        "type": "uint256"
      },
      {
        "name": "detectionMethod",
        "type": "string"
      },
      {
        "name": "severity",
        "type": "string"
      },
      {
        "name": "details",
        "type": "string"
      }
    ],
    "outputs": [],
    "stateMutability": "nonpayable"
  },
  {
    "type": "function",
    "name": "getByzantineEvents",
    "inputs": [
      {
        "name": "nodeIdHash",
        "type": "bytes32"
      }
    ],
    "outputs": [
      {
        "type": "tuple[]",
        "components": [
          {
            "name": "nodeIdHash",
            "type": "bytes32"
          },
          {
            "name": "roundNum",
            "type": "uint256"
          },
          {
            "name": "detectionMethod",
            "type": "string"
          },
          {
            "name": "severity",
            "type": "string"
          },
          {
            "name": "details",
            "type": "string"
          },
          {
            "name": "timestamp",
            "type": "uint256"
          },
          {
            "name": "eventId",
            "type": "uint256"
          }
        ]
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "function",
    "name": "getByzantineEventsByRound",
    "inputs": [
      {
        "name": "nodeIdHash",
        "type": "bytes32"
      },
      {
        "name": "roundNum",
        "type": "uint256"
      }
    ],
    "outputs": [
      {
        "type": "tuple[]",
        "components": [
          {
            "name": "nodeIdHash",
            "type": "bytes32"
          },
          {
            "name": "roundNum",
            "type": "uint256"
          },
          {
            "name": "detectionMethod",
            "type": "string"
          },
          {
            "name": "severity",
            "type": "string"
          },
          {
            "name": "details",
            "type": "string"
          },
          {
            "name": "timestamp",
            "type": "uint256"
          },
          {
            "name": "eventId",
            "type": "uint256"
          }
        ]
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "function",
    "name": "getReputation",
    "inputs": [
      {
        "name": "nodeIdHash",
        "type": "bytes32"
      }
    ],
    "outputs": [
      {
        "name": "totalGradientsSubmitted",
        "type": "uint256"
      },
      {
        "name": "totalCreditsEarned",
        "type": "uint256"
      },
      {
        "name": "byzantineEventCount",
        "type": "uint256"
      },
      {
        "name": "averagePogqScore",
        "type": "uint256"
      },
      {
        "name": "lastActivityTimestamp",
        "type": "uint256"
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "function",
    "name": "getStats",
    "inputs": [],
    "outputs": [
      {
        "type": "uint256"
      },
      {
        "type": "uint256"
      },
      {
        "type": "uint256"
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "function",
    "name": "getVersion",
    "inputs": [],
    "outputs": [
      {
        "type": "string"
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "function",
    "name": "transferOwnership",
    "inputs": [
      {
        "name": "newOwner",
        "type": "address"
      }
    ],
    "outputs": [],
    "stateMutability": "nonpayable"
  },
  {
    "type": "function",
    "name": "owner",
    "inputs": [],
    "outputs": [
      {
        "type": "address"
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "function",
    "name": "version",
    "inputs": [],
    "outputs": [
      {
        "type": "string"
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "function",
    "name": "totalGradients",
    "inputs": [],
    "outputs": [
      {
        "type": "uint256"
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "function",
    "name": "totalCreditsIssued",
    "inputs": [],
    "outputs": [
      {
        "type": "uint256"
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "function",
    "name": "totalByzantineEvents",
    "inputs": [],
    "outputs": [
      {
        "type": "uint256"
      }
    ],
    "stateMutability": "view"
  },
  {
    "type": "event",
    "name": "GradientStored",
    "inputs": [
      {
        "name": "gradientId",
        "type": "string",
        "indexed": True
      },
      {
        "name": "nodeIdHash",
        "type": "bytes32",
        "indexed": True
      },
      {
        "name": "roundNum",
        "type": "uint256",
        "indexed": True
      },
      {
        "name": "pogqScore",
        "type": "uint256",
        "indexed": False
      },
      {
        "name": "zkpocVerified",
        "type": "bool",
        "indexed": False
      }
    ]
  },
  {
    "type": "event",
    "name": "CreditIssued",
    "inputs": [
      {
        "name": "holderHash",
        "type": "bytes32",
        "indexed": True
      },
      {
        "name": "amount",
        "type": "uint256",
        "indexed": False
      },
      {
        "name": "earnedFrom",
        "type": "string",
        "indexed": False
      },
      {
        "name": "creditId",
        "type": "uint256",
        "indexed": False
      }
    ]
  },
  {
    "type": "event",
    "name": "ByzantineEventLogged",
    "inputs": [
      {
        "name": "nodeIdHash",
        "type": "bytes32",
        "indexed": True
      },
      {
        "name": "roundNum",
        "type": "uint256",
        "indexed": True
      },
      {
        "name": "severity",
        "type": "string",
        "indexed": False
      },
      {
        "name": "eventId",
        "type": "uint256",
        "indexed": False
      }
    ]
  },
  {
    "type": "event",
    "name": "ReputationUpdated",
    "inputs": [
      {
        "name": "nodeIdHash",
        "type": "bytes32",
        "indexed": True
      },
      {
        "name": "totalGradients",
        "type": "uint256",
        "indexed": False
      },
      {
        "name": "totalCredits",
        "type": "uint256",
        "indexed": False
      },
      {
        "name": "byzantineEvents",
        "type": "uint256",
        "indexed": False
      }
    ]
  }
]
