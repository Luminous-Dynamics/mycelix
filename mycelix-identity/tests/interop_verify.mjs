import fs from 'node:fs/promises';
import * as vc from '@digitalbazaar/vc';
import {DataIntegrityProof} from '@digitalbazaar/data-integrity';
import {createVerifyCryptosuite} from '@digitalbazaar/eddsa-jcs-2022-cryptosuite';
import {securityLoader} from '@digitalbazaar/security-document-loader';
import {contexts as credentialContexts} from '@digitalbazaar/credentials-context';
import dataIntegrityContext from '@digitalbazaar/data-integrity-context';
import multikeyContext from '@digitalbazaar/multikey-context';

const fixturePath = process.argv[2];
if(!fixturePath) throw new Error('usage: node interop_verify.mjs <fixture>');

const fixture = JSON.parse(await fs.readFile(fixturePath, 'utf8'));
if(fixture.cryptosuite !== 'eddsa-jcs-2022') throw new Error('unexpected cryptosuite');
if(fixture.schema_version !== 1) throw new Error('unexpected fixture schema');

const loader = securityLoader();
for(const [url, document] of credentialContexts) loader.addStatic(url, document);
loader.addStatic(dataIntegrityContext.CONTEXT_URL, dataIntegrityContext.CONTEXT);
loader.addStatic(multikeyContext.CONTEXT_URL, multikeyContext.CONTEXT);

for(const doc of [fixture.issuerDidDocument, fixture.holderDidDocument]) {
  loader.addStatic(doc.id, doc);
  for(const method of doc.verificationMethod ?? []) {
    loader.addStatic(method.id, {
      '@context': multikeyContext.CONTEXT_URL,
      id: method.id,
      type: method.type,
      controller: method.controller,
      publicKeyMultibase: method.publicKeyMultibase
    });
  }
}

const documentLoader = loader.build();
const suite = new DataIntegrityProof({
  cryptosuite: createVerifyCryptosuite()
});

const credentialResult = await vc.verifyCredential({
  credential: fixture.credential,
  suite,
  documentLoader
});
if(!credentialResult.valid) {
  throw new Error('Digital Bazaar rejected Mycelix credential: ' +
    JSON.stringify(credentialResult));
}

const presentationResult = await vc.verify({
  presentation: fixture.presentation,
  challenge: fixture.presentationChallenge,
  suite,
  documentLoader
});
if(!presentationResult.valid) {
  throw new Error('Digital Bazaar rejected Mycelix presentation: ' +
    JSON.stringify(presentationResult));
}

const vpProof = fixture.presentation.proof;
if(vpProof.challenge !== fixture.presentationChallenge ||
   vpProof.domain !== fixture.presentationDomain) {
  throw new Error('fixture challenge/domain mismatch');
}

async function mustReject(label, fn) {
  try {
    const result = await fn();
    if(result?.valid === false) return;
    throw new Error(label + ' unexpectedly verified');
  } catch(error) {
    if(error?.message?.includes('unexpectedly verified')) throw error;
  }
}

const tamperedCredential = structuredClone(fixture.credential);
tamperedCredential.credentialSubject.degree = 'Tampered claim';
await mustReject('tampered credential', () => vc.verifyCredential({
  credential: tamperedCredential,
  suite,
  documentLoader
}));

await mustReject('wrong presentation challenge', () => vc.verify({
  presentation: fixture.presentation,
  challenge: fixture.presentationChallenge + '-wrong',
  suite,
  documentLoader
}));

console.log(JSON.stringify({
  independent_implementation: 'Digital Bazaar',
  cryptosuite: fixture.cryptosuite,
  credential_valid: credentialResult.valid,
  presentation_valid: presentationResult.valid,
  challenge_validated: true,
  domain_present: true,
  tampered_credential_rejected: true,
  wrong_challenge_rejected: true
}, null, 2));
